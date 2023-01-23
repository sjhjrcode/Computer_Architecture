module stimulus ();

   logic  clock;
   logic  In;
   logic  reset_b;
   
   logic  Out;


   logic [31:0]  wd3;

   logic we3; 
   logic [4:0]   ra1, ra2, wa3; 
	//input logic [31:0]  wd3, 
   logic [31:0] rd1, rd2;
   
   integer handle3;
   integer desc3;
   
   // Instantiate DUT
   regfile dut (clock, we3, ra1,ra2,wa3,wd3,rd1,rd2);

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
	#5 $fdisplay(desc3, "Write 1: %b Write 2: %b  Write Enable: %b Wide Data: %b|| Registar 1: %b Registar 2: %b", 
		     ra1, ra2, wa3, wd3, rd1, rd2);
     end   
   
   initial 
     begin      
	#0  we3 = 1'b0;
	#12 we3 = 1'b1;
     #12 wd3 = 32'b11111;
     #12 wa3 = 5'b00000;
     #12 ra1 = 5'b00000;
     #12 wa3 = 5'b00001;
     #12 ra1 = 5'b00001;
     #12 wa3 = 5'b00010;
     #12 ra1 = 5'b00010;
     #12 wa3 = 5'b00100;
     #12 ra1 = 5'b00100;
     #12 wa3 = 5'b01000;
     #12 ra1 = 5'b01000;
     #12 wa3 = 5'b10000;
     #12 ra1 = 5'b10000;
     #12 wa3 = 5'b00011;
     #12 ra1 = 5'b00011;
     #12 wa3 = 5'b11111;
     #12 ra1 = 5'b11111;
     #12 wa3 = 5'b11000;
     #12 ra1 = 5'b11000;
     #12 wa3 = 5'b11100;
     #12 ra1 = 5'b11100;
     #12 wa3 = 5'b11110;
     #12 ra1 = 5'b11110;
     #12 wa3 = 5'b10101;
     #12 ra1 = 5'b10101;
     #12 wa3 = 5'b10011;
     #12 ra1 = 5'b10011;
     #12 wa3 = 5'b10110;
     #12 ra1 = 5'b10110;
     #12 wa3 = 5'b11001;
     #12 ra1 = 5'b11001;
     #12 wa3 = 5'b11011;
     #12 ra1 = 5'b11011;
     #12 ra2 = 5'b00000;
     #12 ra2 = 5'b00001;
     #12 ra2 = 5'b00010;
     #12 ra2 = 5'b00100;
     #12 ra2 = 5'b01000;
     #12 ra2 = 5'b10000;
     #12 ra2 = 5'b00011;
     #12 ra2 = 5'b11111;
     #12 ra2 = 5'b11000;
     #12 ra2 = 5'b11100;
     #12 ra2 = 5'b11110;
     #12 ra2 = 5'b10101;
     #12 ra2 = 5'b10011;
     #12 ra2 = 5'b10110;
     #12 ra2 = 5'b11001;
     #12 ra2 = 5'b11011;
	#12 we3 = 1'b0;
     #12 wd3 = 32'b00000;	


     end

endmodule // regfile_tb

