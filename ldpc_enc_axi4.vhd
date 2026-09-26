--------------------------------------------------------------------------------
--   -----------------------------------------------------------------------
--   Nom du fichier     : LDPC_Encodeur_AXI4.vhd
--   Module             : LDPC_Encodeur_AXI4
--   Version            : 1.0
--   Description        : Wrapper AXI4-Stream 32 bits pour l'encodeur LDPC.
--                        Utilise un COREFIFO_C0 (32 bits W / 1 bit R) genere
--                        sous Libero pour deserialiser l'entree.
--   -----------------------------------------------------------------------
--   Hypotheses :
--     - Le COREFIFO_C0 est un FIFO natif synchrone (non-FWFT).
--       => Q est valide 1 cycle apres RE.
--     - La trame d'entree fait exactement C_DATA_LENGTH_00 bits
--       (= C_SLOT_DATA_LENGTH = 5760 bits = 180 mots de 32 bits).
--     - La sortie de l'encodeur fait egalement C_DATA_LENGTH_00 bits
--       (les deux ratios produisent un mot de code de meme longueur).
--   -----------------------------------------------------------------------
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.all;
use work.LDPC_pkg.all;


entity LDPC_Encodeur_AXI4 is
    port
    (
        --------------------------------
        -- Clock & Reset (AXI)
        --------------------------------
        aclk            : in  std_logic                                  ;
        aresetn         : in  std_logic                                  ; -- Actif bas

        --------------------------------
        -- Parametres
        --------------------------------
        i_ratio         : in  std_logic_vector(C_RATIO_SIZE-1 downto 0)  ; -- 0 = ratio 1, 1 = ratio 1/2
        i_clear         : in  std_logic                                  ; -- Clear general

        --------------------------------
        -- AXI4-Stream Slave (32 bits)
        --------------------------------
        s_axis_tdata    : in  std_logic_vector(31 downto 0)              ;
        s_axis_tvalid   : in  std_logic                                  ;
        s_axis_tready   : out std_logic                                  ;
        s_axis_tlast    : in  std_logic                                  ;

        --------------------------------
        -- AXI4-Stream Master (32 bits)
        --------------------------------
        m_axis_tdata    : out std_logic_vector(31 downto 0)              ;
        m_axis_tvalid   : out std_logic                                  ;
        m_axis_tready   : in  std_logic                                  ;
        m_axis_tlast    : out std_logic
    );
end entity LDPC_Encodeur_AXI4;


architecture rtl of LDPC_Encodeur_AXI4 is

    ---------------------------------------------------------------------------
    -- Constantes locales
    ---------------------------------------------------------------------------
    constant C_TOTAL_BITS    : natural := C_DATA_LENGTH_00              ; -- 5760
    constant C_TOTAL_WORDS   : natural := C_DATA_LENGTH_00 / 32         ; -- 180
    constant C_LAST_WORD_IDX : natural := C_TOTAL_WORDS - 1             ; -- 179

    ---------------------------------------------------------------------------
    -- COREFIFO_C0 (Microchip Libero, config 32 in / 1 out)
    ---------------------------------------------------------------------------
    component COREFIFO_C0
        port
        (
            CLK     : in  std_logic                                 ;
            RESET_N : in  std_logic                                 ;
            WE      : in  std_logic                                 ;
            DATA    : in  std_logic_vector(31 downto 0)             ;
            RE      : in  std_logic                                 ;
            Q       : out std_logic_vector(0 downto 0)              ;
            FULL    : out std_logic                                 ;
            EMPTY   : out std_logic                                 ;
            AFULL   : out std_logic
        );
    end component;

    ---------------------------------------------------------------------------
    -- LDPC_Encodeur
    ---------------------------------------------------------------------------
    component LDPC_Encodeur
        port
        (
            Clk         : in  std_logic                                 ;
            Reset_n     : in  std_logic                                 ;
            i_ratio     : in  std_logic_vector(C_RATIO_SIZE-1 downto 0) ;
            i_clear     : in  std_logic                                 ;
            i_data      : in  std_logic                                 ;
            i_valid     : in  std_logic                                 ;
            i_start     : in  std_logic                                 ;
            i_end       : in  std_logic                                 ;
            o_ready     : out std_logic                                 ;
            o_data      : out std_logic                                 ;
            o_valid     : out std_logic                                 ;
            i_request   : in  std_logic                                 ;
            o_available : out std_logic
        );
    end component;

    ---------------------------------------------------------------------------
    -- Signaux FIFO
    ---------------------------------------------------------------------------
    signal fifo_we      : std_logic                                     ;
    signal fifo_re      : std_logic                                     ;
    signal fifo_data    : std_logic_vector(31 downto 0)                 ;
    signal fifo_q       : std_logic_vector(0 downto 0)                  ;
    signal fifo_full    : std_logic                                     ;
    signal fifo_empty   : std_logic                                     ;
    signal fifo_afull   : std_logic                                     ;

    -- Pipeline de lecture : Q est valide 1 cycle apres RE
    signal fifo_re_d    : std_logic                                     ;
    signal fifo_rdv     : std_logic                                     ; -- Q valide ce cycle
    signal fifo_rdq     : std_logic                                     ;

    ---------------------------------------------------------------------------
    -- Interface encodeur
    ---------------------------------------------------------------------------
    signal enc_ready    : std_logic                                     ;
    signal enc_i_data   : std_logic                                     ;
    signal enc_i_valid  : std_logic                                     ;
    signal enc_i_start  : std_logic                                     ;
    signal enc_i_end    : std_logic                                     ;
    signal enc_data_out : std_logic                                     ;
    signal enc_valid_out: std_logic                                     ;
    signal enc_available: std_logic                                     ;
    signal enc_request  : std_logic                                     ;

    ---------------------------------------------------------------------------
    -- Suivi de trame cote entree
    ---------------------------------------------------------------------------
    signal in_bit_cnt   : unsigned(12 downto 0)                         ; -- 0..5760

    ---------------------------------------------------------------------------
    -- Packing de sortie
    ---------------------------------------------------------------------------
    signal shift_reg    : std_logic_vector(31 downto 0)                 ;
    signal shift_cnt    : unsigned(5 downto 0)                          ; -- 0..32
    signal in_flight    : unsigned(3 downto 0)                          ; -- requetes en cours
    signal out_word_cnt : unsigned(7 downto 0)                          ; -- 0..180
    signal out_word_val : std_logic                                     ;
    signal out_word_dat : std_logic_vector(31 downto 0)                 ;
    signal out_word_lst : std_logic                                     ;

begin

    ---------------------------------------------------------------------------
    -- AXI slave -> FIFO write
    ---------------------------------------------------------------------------
    s_axis_tready <= not fifo_full;
    fifo_we       <= s_axis_tvalid and (not fifo_full);
    fifo_data     <= s_axis_tdata;

    ---------------------------------------------------------------------------
    -- Instance COREFIFO_C0
    ---------------------------------------------------------------------------
    inst_fifo : COREFIFO_C0
        port map
        (
            CLK     => aclk         ,
            RESET_N => aresetn      ,
            WE      => fifo_we      ,
            DATA    => fifo_data    ,
            RE      => fifo_re      ,
            Q       => fifo_q       ,
            FULL    => fifo_full    ,
            EMPTY   => fifo_empty   ,
            AFULL   => fifo_afull
        );

    ---------------------------------------------------------------------------
    -- Instance LDPC_Encodeur
    ---------------------------------------------------------------------------
    inst_enc : LDPC_Encodeur
        port map
        (
            Clk         => aclk             ,
            Reset_n     => aresetn          ,
            i_ratio     => i_ratio          ,
            i_clear     => i_clear          ,
            i_data      => enc_i_data       ,
            i_valid     => enc_i_valid      ,
            i_start     => enc_i_start      ,
            i_end       => enc_i_end        ,
            o_ready     => enc_ready        ,
            o_data      => enc_data_out     ,
            o_valid     => enc_valid_out    ,
            i_request   => enc_request      ,
            o_available => enc_available
        );

    ---------------------------------------------------------------------------
    -- FIFO read -> encoder
    -- Non-FWFT : Q valide 1 cycle apres RE.
    ---------------------------------------------------------------------------
    fifo_re_d  <= fifo_re;
    fifo_rdv   <= fifo_re_d;
    fifo_rdq   <= fifo_q(0);
    enc_i_data <= fifo_rdq;

    PRO_INPUT : process(aclk, aresetn)
    begin
        if aresetn = '0' then
            fifo_re     <= '0';
            enc_i_valid <= '0';
            enc_i_start <= '0';
            enc_i_end   <= '0';
            in_bit_cnt  <= (others => '0');

        elsif rising_edge(aclk) then
            if i_clear = '1' then
                fifo_re     <= '0';
                enc_i_valid <= '0';
                enc_i_start <= '0';
                enc_i_end   <= '0';
                in_bit_cnt  <= (others => '0');
            else
                -- Valeurs par defaut
                enc_i_valid <= '0';
                enc_i_start <= '0';
                enc_i_end   <= '0';
                fifo_re     <= '0';

                -- Requete de lecture si FIFO non vide et encodeur pret
                if fifo_empty = '0' and enc_ready = '1' then
                    fifo_re <= '1';
                end if;

                -- Le bit est disponible le cycle suivant (fifo_rdv = '1')
                if fifo_rdv = '1' and enc_ready = '1' then
                    enc_i_valid <= '1';

                    if in_bit_cnt = 0 then
                        enc_i_start <= '1';
                    end if;

                    if in_bit_cnt = C_TOTAL_BITS - 1 then
                        enc_i_end  <= '1';
                        in_bit_cnt <= (others => '0');
                    else
                        in_bit_cnt <= in_bit_cnt + 1;
                    end if;
                end if;
            end if;
        end if;
    end process PRO_INPUT;

    ---------------------------------------------------------------------------
    -- Encodeur -> packing 32 bits -> AXI master
    ---------------------------------------------------------------------------
    m_axis_tdata  <= out_word_dat;
    m_axis_tvalid <= out_word_val;
    m_axis_tlast  <= out_word_lst;

    -- On limite le nombre de requetes pour ne pas depasser la capacite
    -- du shifter (32 bits) + les bits en vol (latence FIFO interne encodeur).
    enc_request <= '1' when (enc_available = '1' and
                             (to_integer(shift_cnt) + to_integer(in_flight)) < 32)
                   else '0';

    PRO_OUTPUT : process(aclk, aresetn)
        variable v_shift_reg : std_logic_vector(31 downto 0) ;
        variable v_shift_cnt : natural range 0 to 32         ;
        variable v_in_flight : natural range 0 to 15         ;
        variable v_word_cnt  : natural range 0 to 255        ;
        variable v_out_val   : std_logic                     ;
        variable v_out_dat   : std_logic_vector(31 downto 0) ;
        variable v_out_lst   : std_logic                     ;
    begin
        if aresetn = '0' then
            shift_reg    <= (others => '0');
            shift_cnt    <= (others => '0');
            in_flight    <= (others => '0');
            out_word_cnt <= (others => '0');
            out_word_val <= '0';
            out_word_dat <= (others => '0');
            out_word_lst <= '0';

        elsif rising_edge(aclk) then
            -- Chargement des variables depuis les signaux
            v_shift_reg := shift_reg;
            v_shift_cnt := to_integer(shift_cnt);
            v_in_flight := to_integer(in_flight);
            v_word_cnt  := to_integer(out_word_cnt);
            v_out_val   := out_word_val;
            v_out_dat   := out_word_dat;
            v_out_lst   := out_word_lst;

            if i_clear = '1' then
                v_shift_reg := (others => '0');
                v_shift_cnt := 0;
                v_in_flight := 0;
                v_word_cnt  := 0;
                v_out_val   := '0';
                v_out_dat   := (others => '0');
                v_out_lst   := '0';
            else
                -- Suivi des requetes en vol (latence FIFO interne encodeur)
                if    enc_request = '1' and enc_valid_out = '0' then
                    v_in_flight := v_in_flight + 1;
                elsif enc_request = '0' and enc_valid_out = '1' then
                    v_in_flight := v_in_flight - 1;
                end if;

                -- Decalage d'un nouveau bit dans le registre (MSB en premier)
                if enc_valid_out = '1' then
                    v_shift_reg := v_shift_reg(30 downto 0) & enc_data_out;
                    v_shift_cnt := v_shift_cnt + 1;
                end if;

                -- Presentation d'un mot complet au maitre AXI
                if v_out_val = '0' or m_axis_tready = '1' then
                    if v_shift_cnt = 32 then
                        v_out_dat   := v_shift_reg;
                        v_out_val   := '1';
                        v_out_lst   := '1' when v_word_cnt = C_LAST_WORD_IDX else '0';
                        v_shift_cnt := 0;
                        v_word_cnt  := v_word_cnt + 1;
                    elsif v_out_val = '1' and m_axis_tready = '1' then
                        v_out_val := '0';
                    end if;
                end if;
            end if;

            -- Restitution des variables vers les signaux
            shift_reg    <= v_shift_reg;
            shift_cnt    <= to_unsigned(v_shift_cnt, shift_cnt'length);
            in_flight    <= to_unsigned(v_in_flight, in_flight'length);
            out_word_cnt <= to_unsigned(v_word_cnt,  out_word_cnt'length);
            out_word_val <= v_out_val;
            out_word_dat <= v_out_dat;
            out_word_lst <= v_out_lst;
        end if;
    end process PRO_OUTPUT;

end architecture rtl;
