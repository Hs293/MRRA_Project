module manipulatorSolver #(
    parameter                           [7 : 0]         L1  =   10,
    parameter                           [7 : 0]         L2  =   10
)(
    input       wire                                    i_clk,
    input       wire                                    i_reset,
    input       wire                    [7  : 0]        i_coord_x,
    input       wire                    [7  : 0]        i_coord_y,
    output      wire        signed      [31 : 0]        o_q_theta_1,
    output      wire        signed      [31 : 0]        o_q_theta_2
);

    ////////////////////////////////////////////////////////////////////////
    // Parameter Declaration
    // Wire Declaration
    
        localparam  signed      [15 : 0]    Q_RAD_2_DEG = 16'd14667;
        localparam  signed      [15 : 0]    Q_2PI       = 16'd1608;


            wire    signed  [15 : 0]    w_q_coord_x;
            wire    signed  [15 : 0]    w_q_coord_y;
            wire    signed  [15 : 0]    w_q_L1;
            wire    signed  [15 : 0]    w_q_L2;
            wire    signed  [31 : 0]    w_q_coord_x_sq;
            wire    signed  [31 : 0]    w_q_coord_y_sq;
            wire    signed  [31 : 0]    w_q_L1_sq;
            wire    signed  [31 : 0]    w_q_L2_sq;
            wire    signed  [31 : 0]    w_sq_numerator;
            wire    signed  [63 : 0]    w_sq_numerator_extended;
            wire    signed  [31 : 0]    w_sq_denominator;
            wire    signed  [63 : 0]    w_cos_theta_2_temp;
            wire    signed  [15 : 0]    w_cos_theta_2;

    ////////////////////////////////////////////////////////////////////////
    // Mapping Binary Format to Q8.8 Format

        Q8_8_Mapper                         Q_COORD_X(
            .i_binVal                       (i_coord_x),
            .o_qVal                         (w_q_coord_x)
        );
        
        Q8_8_Mapper                         Q_COORD_Y(
            .i_binVal                       (i_coord_y),
            .o_qVal                         (w_q_coord_y)
        );
        
        Q8_8_Mapper                         Q_L1(
            .i_binVal                       (L1),
            .o_qVal                         (w_q_L1)
        );
        
        Q8_8_Mapper                         Q_L2(
            .i_binVal                       (L2),
            .o_qVal                         (w_q_L2)
        );

    ////////////////////////////////////////////////////////////////////////
    // Calculating T1 = cos(Theta2) = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2)
        assign w_q_coord_x_sq           = w_q_coord_x*w_q_coord_x;
        assign w_q_coord_y_sq           = w_q_coord_y*w_q_coord_y;

        assign w_q_L1_sq                = w_q_L1*w_q_L1;
        assign w_q_L2_sq                = w_q_L2*w_q_L2;

        assign w_sq_numerator           = w_q_coord_x_sq + w_q_coord_y_sq - w_q_L1_sq - w_q_L2_sq;
        assign w_sq_denominator         = (w_q_L1 * w_q_L2) << 1;   // *2?
        assign w_sq_numerator_extended  = w_sq_numerator << 16;     //extend? for q format

        assign w_cos_theta_2_temp       = (w_sq_denominator != 0) ? (w_sq_numerator_extended / w_sq_denominator) : (0);
        assign w_cos_theta_2            = w_cos_theta_2_temp[23 : 8];  //cutting? 연산끝나면 8.8비트만 쓰게 파형보면서 이해

    ////////////////////////////////////////////////////////////////////////
    // Calculating Theta2 = arccos(T1)

                wire    signed  [15 : 0]    w_q_theta_2_rad;

        arccos_LUT                          ARCCOS(
            .i_reset                        (i_reset),
            .i_cosVal_q8_8                  (w_cos_theta_2),
            .o_radVal_q8_8                  (w_q_theta_2_rad)
        );

    ////////////////////////////////////////////////////////////////////////
    // Calculating sin(Theta2)

                wire    signed  [15 : 0]    w_q_sin_theta_2;

        sin_LUT                             SIN(
            .i_reset                        (i_reset),
            .i_radVal_q8_8                  (w_q_theta_2_rad),
            .o_sinVal_q8_8                  (w_q_sin_theta_2)
        );

    ////////////////////////////////////////////////////////////////////////
    // Calculating Y / X

                wire    signed  [31 : 0]    w_q_coord_y_extended;
                wire    signed  [31 : 0]    w_q_y_over_x_temp;
                wire    signed  [15 : 0]    w_q_y_over_x;

        assign w_q_coord_y_extended = (w_q_coord_y << 16);
        assign w_q_y_over_x_temp = (w_q_coord_x != 0) ? (w_q_coord_y_extended / w_q_coord_x) : (0);
        assign w_q_y_over_x = w_q_y_over_x_temp[23 : 8];

    ////////////////////////////////////////////////////////////////////////
    // Calculating L2 * sin(Theta2)
    
                wire    signed  [31 : 0]    w_sin_theta_2_numerator;

        assign w_sin_theta_2_numerator = w_q_sin_theta_2 * w_q_L2;

    ////////////////////////////////////////////////////////////////////////
    // Calculating cos(Theta2) * L2 + L1
    
                wire    signed  [31 : 0]    w_cos_theta_2_denominator;
                wire    signed  [31 : 0]    w_q_L1_extended;

        assign w_q_L1_extended = w_q_L1 << 8;
        assign w_cos_theta_2_denominator = w_cos_theta_2 * w_q_L2 + w_q_L1_extended;

    ////////////////////////////////////////////////////////////////////////
    // Calculating T2 = L2 * sin(Theta2) / cos(Theta2) * L2 + L1

                wire    signed  [31 : 0]    w_T2_temp;
                wire    signed  [63 : 0]    w_sin_theta_2_numerator_extended;
                wire    signed  [15 : 0]    w_T2;

        assign w_sin_theta_2_numerator_extended = w_sin_theta_2_numerator << 16;
        assign w_T2_temp = w_sin_theta_2_numerator_extended / w_cos_theta_2_denominator;
        assign w_T2 = w_T2_temp[23 : 8];
    
    ////////////////////////////////////////////////////////////////////////
    // Calculating arcTan(Y / X)

                wire    signed  [15 : 0]    w_arctan_y_over_x;

        arctan_LUT                          ARCTAN_Y_OVER_X(
            .i_reset                        (i_reset),
            .i_tanVal_q8_8                  (w_q_y_over_x),
            .o_radVal_q8_8                  (w_arctan_y_over_x)
        );

    ////////////////////////////////////////////////////////////////////////
    // Calculating arcTan(T2) = arcTan(L2 * sin(Theta2) / (cos(Theta2) * L2 + L1))

                wire    signed  [15 : 0]    w_arctan_T2;
    
        arctan_LUT                          ARCTAN_T2(
            .i_reset                        (i_reset),
            .i_tanVal_q8_8                  (w_T2),
            .o_radVal_q8_8                  (w_arctan_T2)
        );

    ////////////////////////////////////////////////////////////////////////
    // Calculating Theta1 = arcTan(Y / X) - arcTan(L2 * sin(Theta2) / (cos(Theta2) * L2 + L1))

                wire    signed  [15 : 0]    w_q_theta_1_rad;

        assign w_q_theta_1_rad = w_arctan_y_over_x - w_arctan_T2;

    ////////////////////////////////////////////////////////////////////////
    // Mapping radian to degree, deg = rad * 180 / PI = rad * 57.29296875

                wire    signed  [63 : 0]    w_q_theta_1_deg_temp;
                wire    signed  [63 : 0]    w_q_theta_2_deg_temp;

                wire    signed  [31 : 0]    w_q_theta_1_deg;
                wire    signed  [31 : 0]    w_q_theta_2_deg;

        assign w_q_theta_1_deg_temp = w_q_theta_1_rad * Q_RAD_2_DEG << 16;
        assign w_q_theta_2_deg_temp = w_q_theta_2_rad * Q_RAD_2_DEG << 16;

        assign w_q_theta_1_deg = {w_q_theta_1_deg_temp[63 - 16 -: 16], w_q_theta_1_deg_temp[31 -: 16]}; //
        assign w_q_theta_2_deg = {w_q_theta_2_deg_temp[63 - 16 -: 16], w_q_theta_2_deg_temp[31 -: 16]}; //

    ////////////////////////////////////////////////////////////////////////
    // Mapping Degree of Q Value to Degree of Decimal Value

                wire    signed  [15 : 0]    w_theta_1_deg;
                wire    signed  [15 : 0]    w_theta_2_deg;

        BIN_Mapper                              #(
            .INTEGER_WIDTH                      (16)
        )                                       THETA_1_Q_TO_BIN(
            .i_qVal                             (w_q_theta_1_deg),
            .o_binVal                           (w_theta_1_deg)
        );
        
        BIN_Mapper                              #(
            .INTEGER_WIDTH                      (16)
        )                                       THETA_2_Q_TO_BIN(
            .i_qVal                             (w_q_theta_2_deg),
            .o_binVal                           (w_theta_2_deg)
        );

    assign o_q_theta_1 = w_q_theta_1_deg;
    assign o_q_theta_2 = w_q_theta_2_deg;
    ////////////////////////////////////////////////////////////////////////
endmodule
