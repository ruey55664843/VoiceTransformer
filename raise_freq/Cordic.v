module cordic(clk,clk_cal,rst,fft1_data,fft1_valid,freq1,fft1_fin,
    raise_valid,raise_fin,raise_data);

parameter width = 32;
parameter width2 = 16;

input clk;
input clk_cal;
input rst;
input fft1_valid,fft1_fin;
input signed [width-1:0] fft1_data;
input [4:0] freq1;
output raise_valid, raise_fin;
output [31:0] raise_data;

reg signed [width2-1:0] cur_r1,cur_r1_r,cur_r1_w;
reg signed [width2-1:0] cur_i1,cur_i1_r,cur_i1_w;
reg [4:0] cur_freq1;
reg [1:0] quar1;

reg [2:0] state; 
reg [2:0] n_state;

reg refresh_R,refresh_W,refresh_r,refresh_w;    //new input coming
reg fin_r,fin_w;
reg valid_r,valid_w;

reg [3:0] count_precision_r,count_precision_w;
reg signed [15:0] angle_sum1_r,angle_sum1_w;//phase of polar coor.
reg signed [15:0] r1_r,r1_w;//radius of polar coor.
reg [31:0] raised_output_r,raised_output_w;
reg [15:0] cur_phi_r,cur_phi_w;

reg [3:0]fps_counter_r,fps_counter_w;
reg signed [15:0] output_mag_cal_r [0:15];
reg signed [15:0] output_mag_cal_w [0:15];
reg [1:0] output_mag_r [0:15];
reg [1:0] output_mag_w [0:15];
reg [15:0] max_mag_r,max_mag_w;

reg signed [15:0] output_debug_r,output_debug_w;
integer i;

//constant
wire signed [15:0] atan_table [0:7];
assign atan_table[0] = 16'd2880;
assign atan_table[1] = 16'd1700;
assign atan_table[2] = 16'd898;
assign atan_table[3] = 16'd456;
assign atan_table[4] = 16'd229;
assign atan_table[5] = 16'd115;
assign atan_table[6] = 16'd57;
assign atan_table[7] = 16'd29;

reg [8:0] len_scale = 9'd39;
reg [4:0] fft_startfreq = 5'd0;

parameter check_quar = 3'd0;
parameter to_polar = 3'd1;
parameter find_fps_max = 3'd2;
parameter output_ready  = 3'd3;
parameter error = 3'd4;

//next-state logic
always @(*)begin
    n_state = error;
    cur_r1_w = cur_r1_r;
    cur_i1_w = cur_i1_r;
    angle_sum1_w = angle_sum1_r;
    count_precision_w = count_precision_r;
    r1_w = r1_r;
    raised_output_w = raised_output_r;
    refresh_w = refresh_r;
    refresh_W = refresh_R;
    fps_counter_w = fps_counter_r;
    output_debug_w = output_debug_r;
    max_mag_w = max_mag_r;
    valid_w = valid_r;
    fin_w = fin_r;
    for (i=0; i<16; i=i+1)begin
        output_mag_cal_w[i] = output_mag_cal_r[i];
        output_mag_w[i] = output_mag_r[i];
    end

    if ((fft1_valid == 1) && (refresh_r != refresh_R))begin
        cur_r1_w = cur_r1;
        cur_i1_w = cur_i1;
        refresh_w = refresh_R;
        if (cur_freq1 < 16)begin
            n_state = check_quar;
        end
        if (cur_freq1 == fft_startfreq)begin
            if (fps_counter_r < 4'd10)begin
                fps_counter_w = fps_counter_r + 1;
            end
            else begin
                fps_counter_w = 1;
                for (i=0; i<16; i=i+1)begin
                    if (output_mag_cal_r[i]>=(max_mag_r>>2)*2'd3)begin
                        output_mag_w[i] = 2'd3;
                    end
                    else if (output_mag_cal_r[i]>=max_mag_r>>1)begin
                        output_mag_w[i] = 2'd2;
                    end
                    else if (output_mag_cal_r[i]>=max_mag_r>>2)begin
                        output_mag_w[i] = 2'd1;
                    end
                    else begin
                        output_mag_w[i] = 2'd0;
                    end
                end
                if (valid_r==0)begin
                    valid_w = 1;
                end
            end
        end
    end
    else begin
        case (state)
            check_quar:
            begin
                if (cur_r1_r<0)begin
                    if (cur_i1_r>=0)quar1 <= 2;else quar1 <= 3;
                    cur_r1_w = -cur_r1_r;
                end
                else begin
                    quar1 <= 1; //don't care 1 or 4 because same in calculation
                    cur_r1_w = cur_r1_r;
                end
                angle_sum1_w = 0;
                count_precision_w = 0;
                n_state = to_polar;
            end
            to_polar:
            begin
                case(count_precision_r)
                    4'd9:
                    begin
                        n_state = find_fps_max;
                    end
                    4'd8:
                    begin
                        case(quar1)
                            2'd1: angle_sum1_w = angle_sum1_r>>>6;
                            2'd2: angle_sum1_w = ((16'sd11520) - angle_sum1_r)>>>6;
                            2'd3: angle_sum1_w = (-(angle_sum1_r+(16'sd11520)))>>>6;
                        endcase
                        if (cur_r1_r>16'd512)r1_w = (cur_r1_r>>>6)*len_scale;
                        else r1_w = (cur_r1_r*len_scale)>>>6;
                        n_state = state;
                    end
                    default:
                    begin
                        if (cur_i1_r>=0)begin
                            cur_r1_w = cur_r1_r + (cur_i1_r >>> count_precision_r);
                            cur_i1_w = cur_i1_r - (cur_r1_r >>> count_precision_r);
                            angle_sum1_w = angle_sum1_r + atan_table[count_precision_r];
                        end
                        else begin
                            cur_r1_w = cur_r1_r - (cur_i1_r >>> count_precision_r);
                            cur_i1_w = cur_i1_r + (cur_r1_r >>> count_precision_r);
                            angle_sum1_w = angle_sum1_r - atan_table[count_precision_r];
                        end
                        n_state = state;
                    end
                endcase
                count_precision_w = count_precision_r + 1;
            end
            find_fps_max:
            begin
                if (r1_r >= output_mag_cal_r[cur_freq1])begin
                    output_mag_cal_w[cur_freq1] = r1_r;
                end
                if (r1_r >= max_mag_r)begin
                    max_mag_w = r1_r;
                end
                output_debug_w = output_mag_r[cur_freq1];
                n_state = output_ready;
            end
            output_ready:
            begin
                n_state = state;
            end
        endcase
    end
end

assign raise_valid = valid_r;
assign raise_data = {output_mag_r[15],output_mag_r[14],output_mag_r[13],output_mag_r[12],
    output_mag_r[11],output_mag_r[10],output_mag_r[9],output_mag_r[8],
    output_mag_r[7],output_mag_r[6],output_mag_r[5],output_mag_r[4],
    output_mag_r[3],output_mag_r[2],output_mag_r[1],output_mag_r[0]};
assign raise_fin = fin_r;


//seq part of calculation
always @(posedge clk_cal)begin
    if (rst)begin
        state <= 0;
        cur_r1_r <= 0;
        cur_i1_r <= 0;
        angle_sum1_r <= 0;
        count_precision_r <= 0;
        r1_r <= 0;
        raised_output_r <= 0;
        refresh_r <= 0;
        fps_counter_r <= 0;
        output_debug_r <= 0;
        max_mag_r <= 0;
        valid_r <= 0;
        fin_r <= 0;
        for (i=0; i<16; i=i+1)begin
            output_mag_cal_r[i] <= 0;
            output_mag_r[i] <= 0;
        end
    end
    else begin
        state <= n_state;
        cur_r1_r <= cur_r1_w;
        cur_i1_r <= cur_i1_w;
        angle_sum1_r = angle_sum1_w;
        count_precision_r = count_precision_w;
        r1_r <= r1_w;
        raised_output_r <= raised_output_w;
        refresh_r <= refresh_w;
        fps_counter_r <= fps_counter_w;
        output_debug_r <= output_debug_w;
        max_mag_r <= max_mag_w;
        valid_r <= valid_w;
        fin_r <= fin_w;
        for (i=0; i<16; i=i+1)begin
            output_mag_cal_r[i] <= output_mag_cal_w[i];
            output_mag_r[i] <= output_mag_w[i];
        end
    end
end

//seq part of getting data from fft
always @(posedge clk)begin
    if (rst)begin
        cur_r1 <= 0;
        cur_i1 <= 0;
        cur_freq1 <= 63;
        refresh_R <= 0;
    end
    else begin 
        if (fft1_valid == 1'b1)begin
            cur_r1 <= fft1_data[(width-1):(width2)];
            cur_i1 <= fft1_data[(width2-1):0];
            cur_freq1 <= freq1;
            refresh_R <= ~refresh_W;
        end
    end
end

endmodule

