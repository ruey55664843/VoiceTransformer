`timescale 1ns/10ps
`define CYCLE     200.0                // Modify your clock period here
`define CYCLE_CAL     10.0                // Modify your clock period here
`define End_CYCLE  600      // Modify cycle times once your design need more cycle times!

`define fft_fail_limit 48

module tb;

reg         en;
reg         clk;
reg         clk_cal;
reg         rst;
reg [31:0]  fft1_data;
reg         fft1_valid;
reg [4:0]   freq1;
reg         fft1_fin;

wire raise_valid;
wire raise_fin;
wire [31:0] raise_data;

reg [31:0] fft1_mem [0:511];
reg [5:0] fft1_freq_mem [0:511];
initial $readmemh("fft_in1.dat", fft1_mem);
initial $readmemb("fft_freq_in.dat", fft1_freq_mem);

integer i, j ,k, l,count;


raiseFreq DUT(.clk(clk),.clk_cal(clk_cal),.rst(rst),.fft1_data(fft1_data),
    .fft1_valid(fft1_valid),.freq1(freq1),.fft1_fin(fft1_fin),
    .raise_valid(raise_valid),.raise_fin(raise_fin),
    .raise_data(raise_data));

/*
* fir_data: input of fft
* fir_valid: input enable
* fft_data: output of fft
* fft_valid: ?
* freq: ?
* fin: ?
*/


initial begin
    $dumpfile("FAS.fsdb");
    $dumpvars;
end

initial begin
    #0;
    clk         = 1'b0;
    clk_cal     = 1'b0;
    rst       = 1'b0; 
    fft1_fin = 0;
    freq1 = 5'b0;
    en = 0;
    i = 0;   
    j = 0;  
    k = 31;
    l = 0;
    count=0;
end

always begin #(`CYCLE/2) clk = ~clk; end
always begin #(`CYCLE_CAL/2) clk_cal = ~clk_cal; end

initial begin
    #(`CYCLE*0.5)   rst = 1'b1; 
    #(`CYCLE*2); #0.5;   rst = 1'b0; en = 1;
end

// data input & ready
always@(negedge clk ) begin
    if (en) begin
        if (i >= 512 )begin
            fft1_data <= 0;
        end
        else begin
            fft1_data <= fft1_mem[i];
            freq1 <= fft1_freq_mem[i];
            //$display(fft1_mem[i]);
            //$display("\n");
            i <= i + 1;
            if (j<32)begin
                j <= j + 1;
                if (j == 32)begin
                    fft1_fin <= 1;
                end
            end
            else begin
                j <= 1;
                fft1_fin <= 0;
            end
        end
    end
    fft1_valid <= en;
    if(i == 550) begin
        $display("-----------------------------------------------------");
        $display("-------------End of sim ----------------------------");
        $finish;	   
    end   
end




// Terminate the simulation, FAIL
initial  begin
    #(`CYCLE * `End_CYCLE);
    $display("-----------------------------------------------------");
    $display("-------------End of time ----------------------------");
    $finish;
end
endmodule
