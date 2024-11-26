module top_mobileRemoteRobotArm (
    // System
    input       wire                        i_clk,
    input       wire                        i_reset,
    input       wire                        i_enable_sw,

    // UART (Bluetooth)
    input       wire                        i_rx,
    output      wire                        o_tx,

    // Target Coordinates
    output      wire        [7 : 0]         o_fnd,
    output      wire        [3 : 0]         o_fnd_sel,

    // MRRA Joint
    output      wire                        o_shoulder_joint_pwm,
    output      wire                        o_elbow_joint_pwm,

    // MRRA Base
    output      wire        [3 : 0]         o_base_height_phase,
    output      wire                        o_base_rotation_pwm,

    // MRRA End Effector
    output      wire                        o_gripper_joint_pwm
);

    //////////////////////////////  PARAMETERS  ////////////////////////////////////
    
        localparam                          GRIP_MAX_DUTY               =       260;
        localparam                          GRIP_MIN_DUTY               =       80;
        localparam                          GRIP_INC_AMOUNT             =       3;

        parameter                           BASE_MAX_DUTY               =       1000;
        parameter                           BASE_MIN_DUTY               =       0;
        parameter                           BASE_CW_SPEED               =       260;
        parameter                           BASE_CCW_SPEED              =       350;
        parameter                           BASE_DELAY_PER_STEP_US      =       950;        //elevator

        parameter                           BODY_INNER_LINK_LENGTH_CM   =       8;
        parameter                           BODY_OUTER_LINK_LENGTH_CM   =       8;
    
    //////////////////////////////  WIRES  ////////////////////////////////////

                wire                        w_reset;
                wire                        w_grip;
                wire                        w_release;
                wire                        w_inc_height;
                wire                        w_dec_height;
                wire                        w_rot_cw;
                wire                        w_rot_ccw;
                wire        [3 : 0]         w_coord_x;
                wire        [3 : 0]         w_coord_y;

    //////////////////////////////  MRRA CTRL SIGNAL RECEIVER  ////////////////////////////////////

    top_MRRA_rx                             MRRA_RX(
        .i_clk                              (i_clk),
        .i_reset                            (w_reset),
        .i_rx                               (i_rx),
        .o_tx                               (o_tx),
        .o_grip                             (w_grip),
        .o_release                          (w_release),
        .o_inc_height                       (w_inc_height),
        .o_dec_height                       (w_dec_height),
        .o_rot_cw                           (w_rot_cw),
        .o_rot_ccw                          (w_rot_ccw),
        .o_coord_x                          (w_coord_x),
        .o_coord_y                          (w_coord_y),
        .o_fnd                              (o_fnd),
        .o_fnd_sel                          (o_fnd_sel)
    );

    //////////////////////////////  MRRA ACTUATOR  ////////////////////////////////////

    top_MRRA_grip                           #(
        .MAX_DUTY                           (GRIP_MAX_DUTY),
        .MIN_DUTY                           (GRIP_MIN_DUTY),
        .INC_AMOUNT                         (GRIP_INC_AMOUNT)
    )                                       MRRA_END_EFFECTOR(
        .i_clk                              (i_clk),
        .i_reset                            (w_reset),
        .i_grip                             (w_grip),
        .i_release                          (w_release),
        .o_gripper_joint_pwm                (o_gripper_joint_pwm)
    );

    top_MRRA_base                           #(
        .MAX_DUTY                           (BASE_MAX_DUTY),
        .MIN_DUTY                           (BASE_MIN_DUTY),
        .CW_SPEED                           (BASE_CW_SPEED),
        .CCW_SPEED                          (BASE_CCW_SPEED),
        .DELAY_PER_STEP_US                  (BASE_DELAY_PER_STEP_US)
    )                                       MRRA_BASE(
        .i_clk                              (i_clk),
        .i_reset                            (w_reset),
        .i_inc_height                       (w_inc_height),
        .i_dec_height                       (w_dec_height),
        .i_rot_cw                           (w_rot_cw),
        .i_rot_ccw                          (w_rot_ccw),
        .o_base_height_phase                (o_base_height_phase),
        .o_base_rotation_pwm                (o_base_rotation_pwm)
    );

    top_MRRA_body                           #(
        .INNER_LINK_LENGTH_CM               (BODY_INNER_LINK_LENGTH_CM),
        .OUTER_LINK_LENGTH_CM               (BODY_OUTER_LINK_LENGTH_CM)
    )                                       MRRA_BODY(
        .i_clk                              (i_clk),
        .i_reset                            (w_reset),
        .i_coord_x                          (w_coord_x),
        .i_coord_y                          (w_coord_y),
        .o_shoulder_joint_pwm               (o_shoulder_joint_pwm),
        .o_elbow_joint_pwm                  (o_elbow_joint_pwm)
    );

    assign w_reset = i_reset | ~i_enable_sw;
endmodule
//////////////////////////////////////////////////////////////////////////////////////////

module  top_MRRA_rx (
    input       wire                        i_clk,
    input       wire                        i_reset,
    input       wire                        i_rx,
    output      wire                        o_tx,
    output      wire                        o_grip,
    output      wire                        o_release,
    output      wire                        o_inc_height,
    output      wire                        o_dec_height,
    output      wire                        o_rot_cw,
    output      wire                        o_rot_ccw,
    output      wire        [3 : 0]         o_coord_x,
    output      wire        [3 : 0]         o_coord_y,
    output      wire        [7 : 0]         o_fnd,
    output      wire        [3 : 0]         o_fnd_sel
);

                wire        [3 : 0]         w_coord_x;
                wire        [3 : 0]         w_coord_y;

                wire        [15: 0]         w_coord_x_bcd;
                wire        [15: 0]         w_coord_y_bcd;

                wire        [15: 0]         w_fnd_value;

    //////////////////////////////  COMMUNICATION  //////////////////////////////
    commandDecoder                          CMD_DECODER(
        .i_clk                              (i_clk),
        .i_reset                            (i_reset),
        .i_rx                               (i_rx),
        .o_tx                               (o_tx),
        .o_grip                             (o_grip),
        .o_release                          (o_release),
        .o_inc_height                       (o_inc_height),
        .o_dec_height                       (o_dec_height),
        .o_rot_cw                           (o_rot_cw),
        .o_rot_ccw                          (o_rot_ccw),
        .o_coord_x                          (w_coord_x),
        .o_coord_y                          (w_coord_y)
    );
    
    //////////////////////////////  DISPLAY  ////////////////////////////////////
    bin2Dec                                 X_COORD_B2D(
        .i_bin                              ({8'h0, w_coord_x}),
        .o_bcd                              (w_coord_x_bcd)
    );

    bin2Dec                                 Y_COORD_B2D(
        .i_bin                              ({8'h0, w_coord_y}),
        .o_bcd                              (w_coord_y_bcd)
    );

    fndControl                              DISPLAY_COORD(
        .i_clk                              (i_clk),
        .i_reset                            (i_reset),
        .i_value                            (w_fnd_value),
        .o_fnd                              (o_fnd),
        .o_fnd_sel                          (o_fnd_sel)
    );

    assign w_fnd_value = {w_coord_x_bcd[7 -: 8], w_coord_y_bcd[7 -: 8]};
    assign o_coord_x = w_coord_x;
    assign o_coord_y = w_coord_y;

endmodule

module top_MRRA_grip #(
    parameter                               MAX_DUTY            =   1000,
    parameter                               MIN_DUTY            =   0,
    parameter                               INC_AMOUNT          =   3
)(
    input       wire                        i_clk,
    input       wire                        i_reset,
    input       wire                        i_grip,
    input       wire                        i_release,
    output      wire                        o_gripper_joint_pwm
);

    SG90_Grap                               #(
        .MAX_DUTY                           (MAX_DUTY),
        .MIN_DUTY                           (MIN_DUTY),
        .INC_AMOUNT                         (INC_AMOUNT)
    )   GRIP_CTRL                           (
        .i_clk                              (i_clk),
        .i_reset                            (i_reset),
        .i_sw_grap                          (i_grip),
        .i_sw_drop                          (i_release),
        .o_grap_pwm                         (o_gripper_joint_pwm)
    );

endmodule

module top_MRRA_base #(
    parameter                               MAX_DUTY            =   1000,
    parameter                               MIN_DUTY            =   0,
    parameter                               CW_SPEED            =   260,
    parameter                               CCW_SPEED           =   350,
    parameter                               DELAY_PER_STEP_US   =   950
)(
    input       wire                        i_clk,
    input       wire                        i_reset,
    input       wire                        i_inc_height,
    input       wire                        i_dec_height,
    input       wire                        i_rot_cw,
    input       wire                        i_rot_ccw,
    output      wire        [3 : 0]         o_base_height_phase,
    output      wire                        o_base_rotation_pwm
);

    stepperMotorControl                     #(
        .DELAY_PER_STEP_US                  (DELAY_PER_STEP_US)
    )   BASE_HEIGHT_CTRL                    (
        .i_clk                              (i_clk),
        .i_reset                            (i_reset),
        .i_enable                           (i_inc_height | i_dec_height),
        .i_direction                        (i_inc_height),
        .o_phase                            (o_base_height_phase)
    );

    MG90S_360degree                         #(
        .MAX_DUTY                           (MAX_DUTY),
        .MIN_DUTY                           (MIN_DUTY),
        .CW_SPEED                           (CW_SPEED),
        .CCW_SPEED                          (CCW_SPEED)
    )   BASE_ROTATION_CTRL                  (
        .i_clk                              (i_clk),
        .i_reset                            (i_reset),
        .i_sw_CW                            (i_rot_cw),
        .i_sw_CCW                           (i_rot_ccw),
        .o_rotate_pwm                       (o_base_rotation_pwm)
    );

endmodule

module top_MRRA_body #(
    parameter                               INNER_LINK_LENGTH_CM    =   8,
    parameter                               OUTER_LINK_LENGTH_CM    =   8
)(
    input       wire                        i_clk,
    input       wire                        i_reset,
    input       wire        [3 : 0]         i_coord_x,
    input       wire        [3 : 0]         i_coord_y,
    output      wire                        o_shoulder_joint_pwm,
    output      wire                        o_elbow_joint_pwm
);
    
    wire    signed          [31 : 0]        w_q_theta_1;
    wire    signed          [31 : 0]        w_q_theta_2;

    wire                    [11 : 0]        w_shoulder_joint_target_duty;
    wire                    [11 : 0]        w_elbow_joint_target_duty;

    wire                    [11 : 0]        w_shoulder_joint_following_duty;
    wire                    [11 : 0]        w_elbow_joint_following_duty;

    //////////////////////////////  Inverse Kinematics Solver  //////////////////////////
    manipulatorSolver                       #(
        .L1                                 (INNER_LINK_LENGTH_CM),
        .L2                                 (OUTER_LINK_LENGTH_CM)
    )                                       SOLVER(
        .i_reset                            (i_reset),
        .i_coord_x                          ({4'h0, i_coord_x}),
        .i_coord_y                          ({4'h0, i_coord_y}),
        .o_q_theta_1                        (w_q_theta_1),
        .o_q_theta_2                        (w_q_theta_2)
    );

    //////////////////////////////  SHOULDER  ////////////////////////////////////
    SG90_angle2Duty                         #(
        .UP_DOWN_INVERT                     (0),
        .X_OFFSET_TRUE                      (0)
    )                                       SHOULDER_A2D(
        .i_q_theta                          (w_q_theta_1),
        .o_duty                             (w_shoulder_joint_target_duty)
    );

    SG90_dutyTracker                        SHOULDER_DT(
        .i_clk                              (i_clk),
        .i_reset                            (i_reset),
        .i_target_duty                      (w_shoulder_joint_target_duty),
        .o_following_duty                   (w_shoulder_joint_following_duty)
    );

    SG90_Control                            SHOULDER_JOINT(
        .i_clk                              (i_clk),
        .i_reset                            (i_reset),
        .i_duty                             (w_shoulder_joint_following_duty),
        .o_pwm                              (o_shoulder_joint_pwm)
    );

    //////////////////////////////  ELBOW  ////////////////////////////////////
    SG90_angle2Duty                         #(
        .UP_DOWN_INVERT                     (1),
        .X_OFFSET_TRUE                      (1)
    )                                       ELBOW_A2D(
        .i_q_theta                          (w_q_theta_2),
        .o_duty                             (w_elbow_joint_target_duty)
    );

    SG90_dutyTracker                        ELBOW_DT(
        .i_clk                              (i_clk),
        .i_reset                            (i_reset),
        .i_target_duty                      (w_elbow_joint_target_duty),
        .o_following_duty                   (w_elbow_joint_following_duty)
    );
    
    SG90_Control                            ELBOW_JOINT(
        .i_clk                              (i_clk),
        .i_reset                            (i_reset),
        .i_duty                             (w_elbow_joint_following_duty),
        .o_pwm                              (o_elbow_joint_pwm)
    );

endmodule

module commandDecoder(
    input       wire                    i_clk,
    input       wire                    i_reset,
    input       wire                    i_rx,
    output      wire                    o_tx,
    output      wire                    o_grip,
    output      wire                    o_release,
    output      wire                    o_inc_height,
    output      wire                    o_dec_height,
    output      wire                    o_rot_cw,
    output      wire                    o_rot_ccw,
    output      wire        [3 : 0]     o_coord_x,
    output      wire        [3 : 0]     o_coord_y
);

    localparam  DEBUG = 1;

    // Parameters
        localparam          [5 : 0]     IDLE            =       6'b000001;
        localparam          [5 : 0]     SET_X_COORD     =       6'b000010;
        localparam          [5 : 0]     SET_Y_COORD     =       6'b000100;
        localparam          [5 : 0]     SET_HEIGHT      =       6'b001000;
        localparam          [5 : 0]     SET_ROTATION    =       6'b010000;
        localparam          [5 : 0]     SET_GRIP        =       6'b100000;

        localparam          [7 : 0]     GRIP            =       8'h47;  //  "G" 0x47 0b_0100_0111
        localparam          [7 : 0]     RELEASE         =       8'h67;  //  "g" 0x67 0b_0110_0111
        localparam          [7 : 0]     INC_H           =       8'h5A;  //  "Z" 0x5A 0b_0101_1010
        localparam          [7 : 0]     DEC_H           =       8'h7A;  //  "z" 0x7A 0b_0111_1010
        localparam          [7 : 0]     ROT_CW          =       8'h52;  //  "R" 0x52 0b_0101_0010
        localparam          [7 : 0]     ROT_CCW         =       8'h4C;  //  "L" 0x4C 0b_0100_1100
        localparam          [7 : 0]     INC_X           =       8'h58;  //  "X" 0x58 0b_0101_1000
        localparam          [7 : 0]     INC_Y           =       8'h59;  //  "Y" 0x59 0b_0101_1001
        localparam          [7 : 0]     DEC_X           =       8'h78;  //  "x" 0x78 0b_0111_1000
        localparam          [7 : 0]     DEC_Y           =       8'h79;  //  "y" 0x79 0b_0111_1001
        localparam          [7 : 0]     ESC             =       8'h4F;  //  "O" 0x4F 0b_0100_1111

    // Reg & Wires
        wire                [7 : 0]     w_data;
        reg                 [3 : 0]     r_coord_x;
        reg                 [3 : 0]     r_coord_y;

        reg                             r_grip;
        reg                             r_release;
        reg                             r_inc_height;
        reg                             r_dec_height;
        reg                             r_rot_cw;
        reg                             r_rot_ccw;

        reg                 [5 : 0]     present_state;
        reg                 [5 : 0]     next_state;

        wire                            w_clk_usec;
        wire                            w_clk_msec;
        wire                            w_clk_sec;
        wire                [15 : 0]    bcd_x_coord;
        wire                [15 : 0]    bcd_y_coord;

    rx_uart_3_fix                       UART(
        .clk                            (i_clk),
        .reset_p                        (i_reset),
        .i_data                         (i_rx),
        .led                            (w_data)
    );

    clock_div_100_LSW                   i_us_clk(
        .clk                            (i_clk),
        .reset_p                        (i_reset),
        .cp_div_100                     (w_clk_usec)
    );
    
    clock_div_1000_LSW                  i_ms_clk(
        .clk                            (i_clk),
        .reset_p                        (i_reset),
        .clk_source                     (w_clk_usec), 
        .cp_div_1000_nedge              (w_clk_msec)
    );
    
    clock_div_1000_LSW                  i_s_clk(
        .clk                            (i_clk),
        .reset_p                        (i_reset),
        .clk_source                     (w_clk_msec), 
        .cp_div_1000_nedge              (w_clk_sec)
    );


    always @(negedge i_clk or posedge i_reset) begin
        if (i_reset) begin
            present_state <= IDLE;
        end
        else begin
            present_state <= next_state;
        end
    end

    always @(posedge i_clk or posedge i_reset) begin
        if (i_reset) begin
            r_coord_x       <=      8;
            r_coord_y       <=      8;
            r_grip          <=      0;
            r_release       <=      0;
            r_inc_height    <=      0;
            r_dec_height    <=      0;
            r_rot_cw        <=      0;
            r_rot_ccw       <=      0;
            next_state      <=      IDLE;
        end
        else begin
            case(present_state)
                IDLE: begin
                    case (w_data)
                        GRIP        :   next_state <= SET_GRIP;
                        RELEASE     :   next_state <= SET_GRIP;
                        INC_H       :   next_state <= SET_HEIGHT;
                        DEC_H       :   next_state <= SET_HEIGHT;
                        ROT_CW      :   next_state <= SET_ROTATION;
                        ROT_CCW     :   next_state <= SET_ROTATION;
                        INC_X       :   next_state <= SET_X_COORD;
                        INC_Y       :   next_state <= SET_Y_COORD;
                        DEC_X       :   next_state <= SET_X_COORD;
                        DEC_Y       :   next_state <= SET_Y_COORD;
                        ESC         :   next_state <= IDLE;
                        default     :   next_state <= IDLE;
                    endcase
                    r_coord_x       <=      r_coord_x;
                    r_coord_y       <=      r_coord_y;
                    r_grip          <=      0;
                    r_release       <=      0;
                    r_inc_height    <=      0;
                    r_dec_height    <=      0;
                    r_rot_cw        <=      0;
                    r_rot_ccw       <=      0;
                end

                SET_X_COORD: begin
                    if (w_data == ESC) begin
                        next_state <= IDLE;
                    end
                    else begin
                        if (w_clk_sec) begin
                            if      (w_data == INC_X && r_coord_x == 15) begin
                                r_coord_x <= r_coord_x;
                            end
                            else if (w_data == DEC_X && r_coord_x == 1) begin
                                r_coord_x <= r_coord_x;
                            end
                            else if (w_data == INC_X && r_coord_x < 15 && w_coord_enable) begin // X ����
                                r_coord_x <= r_coord_x + 1;
                            end
                            else if (w_data == DEC_X && r_coord_x > 1) begin // x ����
                                r_coord_x <= r_coord_x - 1;
                            end                          
                        end
                    end
                end

                SET_Y_COORD: begin
                    if (w_data == ESC) begin
                        next_state <= IDLE;
                    end
                    else begin
                        if (w_clk_sec) begin
                            if      (w_data == INC_Y && r_coord_y == 15) begin
                                r_coord_y <= r_coord_y;
                            end
                            else if (w_data == DEC_Y && r_coord_y == 0) begin
                                r_coord_y <= r_coord_y;
                            end
                            else if (w_data == INC_Y && r_coord_y < 15 && w_coord_enable) begin
                                r_coord_y <= r_coord_y + 1;
                            end
                            else if (w_data == DEC_Y && r_coord_y > 0) begin
                                r_coord_y <= r_coord_y - 1;
                            end                         
                        end
                    end
                end

                SET_HEIGHT: begin
                    if (w_data == ESC) begin
                        next_state <= IDLE;
                    end
                    else begin
                        if (w_clk_sec) begin
                            if      (w_data == INC_H) begin
                                r_inc_height <= 1;
                                r_dec_height <= 0;
                            end
                            else if (w_data == DEC_H) begin
                                r_inc_height <= 0;
                                r_dec_height <= 1;
                            end
                        end
                    end
                end
                
                SET_GRIP: begin
                    if (w_data == ESC) begin
                        next_state <= IDLE;
                    end
                    else begin
                        if (w_clk_sec) begin
                            if      (w_data == GRIP) begin
                                r_grip      <= 1;
                                r_release   <= 0;
                            end
                            else if (w_data == RELEASE) begin
                                r_grip      <= 0;
                                r_release   <= 1;
                            end
                        end
                    end
                end
                
                SET_ROTATION: begin
                    if (w_data == ESC) begin
                        next_state <= IDLE;
                    end
                    else begin
                        if (w_clk_sec) begin
                            if      (w_data == ROT_CW) begin
                                r_rot_cw    <= 1;
                                r_rot_ccw   <= 0;
                            end
                            else if (w_data == ROT_CCW) begin
                                r_rot_cw    <= 0;
                                r_rot_ccw   <= 1;
                            end
                        end
                    end
                end

                default: next_state <= IDLE;
            endcase
        end
    end

    assign o_tx         = 1'b0;
    assign o_coord_x    = r_coord_x;
    assign o_coord_y    = r_coord_y;
    assign o_grip       = r_grip;
    assign o_release    = r_release;
    assign o_inc_height = r_inc_height;
    assign o_dec_height = r_dec_height;
    assign o_rot_cw     = r_rot_cw;
    assign o_rot_ccw    = r_rot_ccw;
    assign w_coord_enable = ((r_coord_x*r_coord_x + r_coord_y*r_coord_y) <= 225) ? (1'b1) : (1'b0);
endmodule

module rx_uart_3_fix(
    input clk,
    input reset_p,
    input i_data,
    output reg [7:0] led
);
    parameter             B_rate     = 9600;
    parameter             sys_clk    = 100_000_000;
    parameter             T_rate     = sys_clk / B_rate;
    parameter       [2:0] IDLE       = 3'b001;
    parameter       [2:0] RX_MODE    = 3'b010;
    parameter       [2:0] DONE       = 3'b100;

    reg [2:0] state, next_state;
    reg [3:0] bit_count;     // Bit count (start bit, 8 data bits, stop bit)
    reg [7:0] r_data;        // Received data
    reg [15:0] count;        // Timer count
    reg t_pulse;             // Timer pulse
    reg data_ready;          // Data ready flag

    // edge_detector_n module definition needed
    wire i_data_pedge, i_data_nedge;
    wire t_pulse_pedge;

    edge_detector_n_LSW i_data_ed(
        .clk(clk), .reset_p(reset_p), .cp(i_data),
        .p_edge(i_data_pedge), .n_edge(i_data_nedge)
    );
    
    edge_detector_n_LSW T_rate_ed(
        .clk(clk), .reset_p(reset_p), .cp(t_pulse),
        .p_edge(t_pulse_pedge), .n_edge()
    );

    // Timer pulse generation
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            count <= 0;
            t_pulse <= 0;
        end
        else begin
            if (count < (T_rate - 1)/2) begin
                count <= count + 1;
            end
            else begin
                count <= 0;
                t_pulse <= ~t_pulse; // Toggle pulse every half period
            end
        end
    end
    
    // State machine
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            state <= IDLE;
            bit_count <= 0;
            r_data <= 0;
            data_ready <= 0;
        end
        else begin
            case (state)
                IDLE: begin
                    if (i_data_nedge) begin
                        bit_count <= 0;
                        data_ready <= 0;
                        state <= RX_MODE;
                    end
                end

                RX_MODE: begin
                    if (t_pulse_pedge) begin
                        if (bit_count == 0) begin
                            // Detect start bit (this value should be 0)
                            if (i_data == 0) begin
                                bit_count <= bit_count + 1;
                            end
                        end
                        else if (bit_count < 10) begin
                            // Receive data bits
                            if (bit_count >= 1 && bit_count <= 8) begin
                                r_data[bit_count - 1] <= i_data;
                            end
                            bit_count <= bit_count + 1;
                        end
                        else if (bit_count == 10) begin
                            // Verify stop bit (this value should be 1)
                            if (i_data == 1) begin
                                data_ready <= 1;
                            end
                            bit_count <= 0;
                            state <= DONE;
                        end
                    end
                end

                DONE: begin
                    if (data_ready) begin
                        led <= r_data; // Send data to LED
                        state <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule

module clock_div_100_LSW(
    input clk, reset_p,
    output clk_div_100,
    output cp_div_100
    );
    
    reg[6:0] cnt_sysclk;

    always @(negedge clk or posedge reset_p)begin 
        if(reset_p) cnt_sysclk = 1;
        else begin
            if(cnt_sysclk >= 99) cnt_sysclk = 0;
            else cnt_sysclk = cnt_sysclk + 1;
        end
    end
    
    assign cp_div_100 = (cnt_sysclk < 50) ? 0 : 1;
    edge_detector_n_LSW ed(
        .clk(clk), .reset_p(reset_p), .cp(cp_div_100),
        .n_edge(clk_div_100));
           
endmodule

module clock_div_1000_LSW(
    input clk, reset_p,
    input clk_source,
    output cp_div_1000_nedge
    );
    wire nedge_source, cp_div_1000;
    
    reg[9:0] cnt_clk_source;
    
    edge_detector_n_LSW ed(
        .clk(clk), .reset_p(reset_p), .cp(clk_source),
        .n_edge(nedge_source));
            
    always @(negedge clk or posedge reset_p)begin
        if (reset_p) 
            cnt_clk_source = 0;
        else if (nedge_source) begin
            if (cnt_clk_source >= 499) 
                cnt_clk_source = 0;
            else 
                cnt_clk_source = cnt_clk_source + 1;
        end
    end
    
    assign cp_div_1000 = (cnt_clk_source < 250) ? 0 : 1;
    edge_detector_n_LSW ed1000(
        .clk(clk), .reset_p(reset_p), .cp(cp_div_1000),
        .n_edge(cp_div_1000_nedge));
endmodule

module edge_detector_n_LSW(
    input clk,
    input reset_p, cp,
    output p_edge, n_edge);
    
    reg ff_cur, ff_old;
    always @(negedge clk or posedge reset_p)begin
    if(reset_p)begin
        ff_cur <= 0;
        ff_old <= 0; 
    end
    else begin
        ff_old <= ff_cur;
        ff_cur <= cp;    
    end
end

assign p_edge = ({ff_cur, ff_old} == 2'b10) ? 1 : 0;
assign n_edge = ({ff_cur, ff_old} == 2'b01) ? 1 : 0;
endmodule

 module SG90_Grap    #(
    parameter                                           MAX_DUTY    = 1000,
    parameter                                           MIN_DUTY    = 0,
    parameter                                           INC_AMOUNT  = 3
)(
    input	wire                    i_clk,   
    input	wire                    i_reset,  
    input	wire                    i_sw_grap,     // Android Button 1
    input	wire                    i_sw_drop,     // Android Button 2
    output	wire                    o_grap_pwm     
);
    integer                                             clk_div;
    reg             [11 : 0]                            r_duty_angle;
    wire                                                clk_div_nedge;
    reg                                                 flag_grap;
    reg                                                 flag_drop;

    edge_detector_n                                    ed(
        .i_clk                                          (i_clk),
        .i_reset                                        (i_reset),
        .i_cp                                           (clk_div[21]),
        .o_posedge                                      (),
        .o_negedge                                      (clk_div_nedge)
    );

    SG90_Control                                        grap(
        .i_clk                                          (i_clk),
        .i_reset                                        (i_reset),
        .i_duty                                         (r_duty_angle),//Duty�Է� (�� 80~540)
        .o_pwm                                          (o_grap_pwm)//PWM���
    );
    
    always@(posedge i_clk) clk_div = clk_div + 1;//???

    always @(posedge i_clk or posedge i_reset) begin
        if(i_reset) begin
            r_duty_angle =  MIN_DUTY;
            flag_grap = 0;
            flag_drop = 0;
        end
        else if(clk_div_nedge) begin
            if(flag_grap && r_duty_angle < MAX_DUTY) begin//MAX Duty �� 540
                r_duty_angle = r_duty_angle + INC_AMOUNT;//Duty ����, ���� ���� ȸ�� �ӵ��� �޶���
            end
            else if(flag_drop && r_duty_angle > MIN_DUTY) begin//MIN Duty �� 70
                r_duty_angle = r_duty_angle - INC_AMOUNT;//Duty ����
            end
        end
        else begin
            if(i_sw_grap) begin
                flag_grap = 1;
            end
            else if(i_sw_grap == 0) begin
                flag_grap = 0;
            end
            if(i_sw_drop) begin
                flag_drop = 1;
            end
            else if(i_sw_drop == 0) begin
                flag_drop = 0;
            end
        end
    end

endmodule


module MG90S_360degree #(
    parameter                                           MAX_DUTY    = 1000,
    parameter                                           MIN_DUTY    = 0,
    parameter                                           CW_SPEED    = 260,
    parameter                                           CCW_SPEED   = 350
)(
    input	wire                    i_clk,
    input	wire                    i_reset,
    input	wire                    i_sw_CW,    //  Android Button  ��
    input	wire                    i_sw_CCW,    //  Android Button  ��
    output	wire                    o_rotate_pwm
);
    integer                                             clk_div;
    reg             [11 : 0]                            r_duty_angle;
    wire                                                clk_div_nedge;
    reg                                                 flag_CW;
    reg                                                 flag_CCW;
    
    edge_detector_n                                     ed(
        .i_clk                                          (i_clk),
        .i_reset                                        (i_reset),
        .i_cp                                           (clk_div[21]),
        .o_posedge                                      (),
        .o_negedge                                      (clk_div_nedge)
    );

    SG90_Control                                        rotate(
        .i_clk                                          (i_clk),
        .i_reset                                        (i_reset),
        .i_duty                                         (r_duty_angle),//  Duty�Է� (�� 80~540)
        .o_pwm                                          (o_rotate_pwm)//  PWM���
    );

    always@(posedge i_clk) clk_div = clk_div + 1;

    always @(posedge i_clk or posedge i_reset) begin
        if(i_reset) begin
            r_duty_angle = 0;
            flag_CW = 0;
            flag_CCW = 0;
        end
        else if(clk_div_nedge) begin
            if(flag_CW && r_duty_angle < MAX_DUTY) begin
                r_duty_angle = CW_SPEED;//81~290�ð�(CW)
            end
            
            else if(flag_CCW && r_duty_angle >= MIN_DUTY) begin
                r_duty_angle = CCW_SPEED;//320 ~ �ݽð�(CCW)
            end
            
            else if(!flag_CW && !flag_CCW) begin
                r_duty_angle = 0;
            end
            
        end
        else begin
            if(i_sw_CW) begin
                flag_CW = 1;
            end
            else if(i_sw_CW == 0) begin
                flag_CW = 0;
            end
            if(i_sw_CCW) begin
                flag_CCW = 1;
            end
            else if(i_sw_CCW == 0) begin
                flag_CCW = 0;
            end
        end
    end

endmodule
module SG90_dutyTracker (
    input       wire                        i_clk,
    input       wire                        i_reset,
    input       wire            [11 : 0]    i_target_duty,
    output      wire            [11 : 0]    o_following_duty
);

    reg [11 : 0] r_following_duty;
    reg [21 : 0] r_counter;

    wire        w_delay_counter_posedge;

    edge_detector_n                         ED_DELAY_COUNTER(
        .i_clk                              (i_clk),
        .i_reset                            (i_reset),
        .i_cp                               (r_counter[18]),
        .o_posedge                          (w_delay_counter_posedge),
        .o_negedge                          ()
    );

    always @(posedge i_clk or posedge i_reset) begin
        if (i_reset) begin
            r_counter <= 0;
        end
        else begin
            r_counter <= r_counter + 1;
        end
    end

    always @(posedge i_clk or posedge i_reset) begin
        if (i_reset) begin
            r_following_duty <= 0;
        end
        else begin
            if (w_delay_counter_posedge) begin
                if (r_following_duty < i_target_duty) begin
                    r_following_duty <= r_following_duty + 1;
                end
                else if (r_following_duty > i_target_duty) begin
                    r_following_duty <= r_following_duty - 1;
                end
            end
        end
    end

    assign o_following_duty = r_following_duty;

endmodule
