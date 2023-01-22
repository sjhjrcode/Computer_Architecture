module stimulus ();

   logic  clock;
   logic  In;
   logic  reset_b;
   
   logic  Out;



   logic we3; 
   logic [4:0]   ra1, ra2, wa3; 
	//input logic [31:0]  wd3, 
   logic [31:0] rd1, rd2;
   
   integer handle3;
   integer desc3;
   
   // Instantiate DUT
   regfile dut (clock, we3, ra1,ra2,wa3,rd1,rd2);

   // Setup the clock to toggle every 1 time units 
   initial 
     begin	
	clock = 1'b1;
	forever #5 clock = ~clock;
     end

   initial
     begin
	// Gives output file name
	handle3 = $fopen("test.out");
	// Tells when to finish simulation
	#500 $finish;		
     end

   always 
     begin
	desc3 = handle3;
	#5 $fdisplay(desc3, "%b %b %b|| %b %b", 
		     ra1, ra2, wa3, rd1, rd2);
     end   
   
   initial 
     begin      
	#0  we3 = 1'b0;
	#12 we3 = 1'b1;	
	#0  ra1 = 1'b0;
	#20 ra1 = 1'b1;
	#20 ra1 = 1'b0;
     end

endmodule // regfile_tb

