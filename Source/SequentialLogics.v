//////////////////////////////////////////////////////////////////////
module ringCounterFnd(
    input       wire                i_clk,
    input       wire                i_reset,
    output      wire    [3 : 0]     o_fnd_sel
);
    
    reg     [16 : 0]                r_delay_counter;
    reg     [3 : 0]                 r_fnd_sel;
    wire                            w_delay_counter_posedge;
    
    edge_detector_n                 ED_DELAY_COUNTER(
        .i_clk                      (i_clk),
        .i_reset                    (i_reset),
        .i_cp                       (r_delay_counter[16]),
        .o_posedge                  (w_delay_counter_posedge),
        .o_negedge                  ()
    );

    always @(posedge i_clk) begin
        r_delay_counter <= r_delay_counter + 1;
    end
    
    always @(posedge i_clk or posedge i_reset)begin
        if (i_reset) begin
            r_fnd_sel <= 4'b1110;
        end
        else if (w_delay_counter_posedge) begin
            if (r_fnd_sel == 4'b0111) begin
                r_fnd_sel <= 4'b1110;
            end
            else begin
                r_fnd_sel <= {r_fnd_sel[2 : 0], 1'b1};
            end
        end
    end

    assign o_fnd_sel = r_fnd_sel;
endmodule
//////////////////////////////////////////////////////////////////////

module edge_detector_n(
    input       wire        i_clk,
    input       wire        i_reset,
    input       wire        i_cp,
    output      wire        o_posedge,
    output      wire        o_negedge
);

    reg r_cp;
    reg r_cp_z;

    always @(negedge i_clk or posedge i_reset)begin
        if(i_reset)begin
            r_cp    <= 0;
            r_cp_z  <= 0;
        end
        else begin
            r_cp    <= i_cp;
            r_cp_z  <= r_cp;
        end
    end
    
    assign o_posedge    =   r_cp     &   ~r_cp_z;
    assign o_negedge    =   ~r_cp    &   r_cp_z;
endmodule

//////////////////////////////////////////////////////////////////////

