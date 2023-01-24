module regfile (input logic         clk, 
		input logic 	    we3, 
		input logic [4:0]   ra1, ra2, wa3, 
		input logic [31:0]  wd3, 
		output logic [31:0] rd1, rd2);
   
   logic [31:0] 		    rf[31:0];
   // three ported register file
   // read two ports combinationally
   // write third port on rising edge of clock
   // register 0 hardwired to 0
   always_ff @ (posedge clk) 
   begin
      if(we3) begin
         rf[wa3]<=wd3;
 
      end
 
      else  begin
         rf[0]<=32'b0;  
      end
      
      assign rd1 = (ra1==0) ? 32'b0: rf[ra1];
      assign rd2 = (ra2==0) ? 32'b0: rf[ra2]; 
         //assign out = ra1 & ra2;
     
   end   
   
endmodule // regfile
