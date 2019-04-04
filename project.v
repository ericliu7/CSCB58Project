module project(
    CLOCK_50,
    SW,
    KEY,
    HEX0,
    HEX1,
    HEX2,
    HEX3,
    HEX4,
    HEX5,
    HEX6,
    HEX7);
    
    input CLOCK_50;
    input [12:0] SW;
    input [3:0] KEY;
    
    output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7;
    
    // wires that connect datapath and FSM
    wire loadRNum, displayRNum, loadPNum, loadLevel, display_input, addPoint, startTi, gameOver;
    
    wire go, real_reset;

    // wire to clock module
    wire [7:0] ti;

    // wires to HEX
    wire [3:0] point1, point0, numP1, numP0, numR1, numR0;
    
    assign go = ~KEY[0];

    assign real_reset = SW[8] && gameOver;

    // instantiate control module
    control c0(
        .clk(CLOCK_50),
        .resetn(real_reset),
        .go(go),
        .loadRNum(loadRNum),
        .displayRNum(displayRNum),
        .loadPNum(loadPNum),
        .loadLevel(loadLevel),
        .display_input(display_input),
        .addPoint(addPoint),
        .startTi(startTi));
        
    // instantiate datapath module
    datapath d0(
        .clk(CLOCK_50),
        .resetn(real_reset),
        .data_in(SW[7:0]),
        .level(SW[12:9]),
        .loadRNum(loadRNum),
        .displayRNum(displayRNum),
        .loadPNum(loadPNum),
        .loadLevel(loadLevel),
        .display_input(display_input),
        .addPoint(addPoint),
        .startTi(startTi),
        .point1(point1),
        .point0(point0),
        .numP1(numP1),
        .numP0(numP0),
        .numR1(numR1),
        .numR0(numR0),
        .disTi(ti),
        .gameOver(gameOver));

    // instantiate datapath module
    hex_decoder h7(
        .hex_digit(ti[7:4]),
        .segments(HEX7));    

    hex_decoder h6(
        .hex_digit(ti[3:0]),
        .segments(HEX6));

    hex_decoder h5(
        .hex_digit(point1),
        .segments(HEX5));
        
    hex_decoder h4(
        .hex_digit(point0),
        .segments(HEX4));
        
    hex_decoder h3(
        .hex_digit(numP1),
        .segments(HEX3));
        
    hex_decoder h2(
        .hex_digit(numP0),
        .segments(HEX2));
        
    hex_decoder h1(
        .hex_digit(numR1),
        .segments(HEX1));
        
    hex_decoder h0(
        .hex_digit(numR0),
        .segments(HEX0));
endmodule

module control(
	input clk,
	input resetn,
	input go,
    
    output reg loadRNum,
    output reg displayRNum,
    output reg loadPNum,
    output reg loadLevel,
    output reg display_input,
    output reg addPoint,
    output reg startTi);
    
    reg [5:0] current_state, next_state;
    
    localparam 	INI           = 5'd0,
				WAIT_LEVEL    = 5'd1,
                START_TI      = 5'd2,
				GENERATE_RNG  = 5'd3,
                SHOW_RNG      = 5'd4,
                PLAYER_INPUT  = 5'd5, 
                WAIT_INPUT    = 5'd6,
				DISPLAY_INPUT = 5'd7,
                ADD_POINT     = 5'd8,
                END           = 5'd9;

	always@(*)
	begin: state_table
		case (current_state)
			INI: next_state = go ? WAIT_LEVEL : INI; // start state, return to here when reset or gameOver
            WAIT_LEVEL: next_state = go ? WAIT_LEVEL : START_TI; // wait for player to input level and mode he want to play
            START_TI: next_state = GENERATE_RNG; // start to record the time and show it on HEX
            GENERATE_RNG: next_state = SHOW_RNG; // generate the random number
			SHOW_RNG: next_state = PLAYER_INPUT; // show the random number on HEX
            PLAYER_INPUT: next_state = go ? WAIT_INPUT : PLAYER_INPUT; // wait player to input his binary number using switches
            WAIT_INPUT: next_state = go ? WAIT_INPUT : DISPLAY_INPUT; // waiting
            DISPLAY_INPUT: next_state = ADD_POINT; // display the player's input
            ADD_POINT: next_state = END; // add point according to the input and show it on HEX
            END: next_state = GENERATE_RNG; // end state for onr phase, then go to generate another number 
			default: next_state = INI;	
		endcase
	end
    
    // initially set everything to zero, set the according parameter to one according to state
    always@(*)
	begin: make_output
        loadRNum = 0;
        displayRNum = 0;
        loadPNum = 0;
        loadLevel = 0;
        display_input = 0;
        addPoint = 0;
        startTi = 0;
		case(current_state)
			INI: begin
                    loadLevel = 1;
                 end
            START_TI: begin
                    startTi = 1;
                 end
            GENERATE_RNG: begin
                    loadRNum = 1;
                 end
            SHOW_RNG: begin
                    displayRNum = 1;
                 end
            PLAYER_INPUT: begin
                    loadPNum = 1;
                 end
            DISPLAY_INPUT: begin
                    display_input = 1;
                 end
            ADD_POINT: begin
                    addPoint = 1;
                 end
            endcase
    end
    
    // state engine
    always@(posedge clk)
	begin: state_FFs
		if(!resetn) // goto resting if reset
			current_state <= INI;
		else
			current_state <= next_state;
	end

endmodule

module datapath(
    input clk,
    input resetn,
    input [7:0] data_in, 
    input [3:0] level,
    input loadRNum, displayRNum, loadPNum, loadLevel, display_input, addPoint, startTi,
    
    output [3:0] point1, point0,
    output reg [3:0] numP1, numP0, numR1, numR0,
    output [7:0] disTi,
    output reg gameOver);

    reg clEn = 0; // variable controls the start and end of clock

    wire [7:0] ti;

    reg [7:0] disTiReg; // reg for disTi

    reg [3:0] nRN; // number of random number for eight number mode
    reg reS; // signal to reset time in eightnumber mode

    wire re; // reset

    assign disTi = disTiReg; // display time
    assign re = resetn && reS;

    // instantiate clock module
    clock clock0(
        .clk(clk),
        .reset(re),
        .enable(clEn),
        .ti(ti));
        
    // initial value for regs
    reg [7:0] point = 0;
    reg [7:0] randomN = 0;
    reg [7:0] theN = 0;
    reg [7:0] playerN = 0;
    reg [3:0] theL = 0; // when theL[2:1] == 00, ifinity mode, theL[2:1] == 01, countdown mode, theL[2:1] == 10, eightnumber mode. when theL[0] == 0,
    //the difficulty is easy, otherwise hard. If theL[3] = 1, read the score board for that mode, otherwise main game.

    // connect points to HEX
    assign point1 = point[7:4];
    assign point0 = point[3:0];

    // for scoreboard.  We created scoreboard for easy countdown mode, however it is not working. As it won't affect other parts of code, so we keep it here.
    // 
    reg [3:0] scoreCounter = 0;

    reg clk_010, r_or_w_010; // go for memory, and check if it is read or write separately.
    reg [7:0] index_010; // address for memory
    reg [7:0] data_in_010;
    wire [7:0] data_out_010;

    // instantiate memory
    mem5 mem5_010(
        .clk(clk_010),
        .r_or_w(r_or_w_010),
        .index(index_010),
        .data_in(data_in_010),
        .data_out(data_out_010));

    always@(posedge clk) begin
        gameOver = 1;
        reS = 1;
        randomN = randomN + 8'b00000001; // random number generator
        if(!resetn)
            begin
                // initial value forr variables
                point = 0;
                randomN = 0;
                theN = 0;
                playerN = 0;
                theL = 0;
                numP1 = 0;
                numP0 = 0;
                numR1 = 0;
                numR0 = 0;
                clEn = 0;
                gameOver = 1;
                nRN = 0;
            end
        
        if (loadLevel)
            begin
                theL = level; 
            end
        
        if (theL[3] == 0) begin
            if (startTi)
                begin
                    clEn = 1;
                end

            if (theL[2:1] == 2'd1)
                disTiReg = 8'd30 - ti;
            else
                disTiReg = ti;

            // gameover for countdown mode
            if (disTiReg == 8'hff)
                begin
                    // add points to score table
                    data_in_010 = point;
                    r_or_w_010 = 1;
                    clk_010 = 1;
                    clk_010 = 0;
                    
                    // gameover
                    gameOver = 0;
                end

            // gameover for eight number mode
            if (nRN == 4'd8)
                gameOver = 0;
                
            // rules to add point
            if (addPoint)
                begin
                    if (theL[0])
                        begin
                            // for eight number mode, get correct answer within 2 seconds grants 3 points, 5 second for 2 points, else for 1 point, wrong answer for 0 point
                            if (theN == playerN) begin
                                if (theL[2:1] == 2'd2)
                                    begin
                                        if (ti <= 8'd2)
                                            point <= point + 3;
                                        else if (ti <= 8'd5)
                                            point <= point + 2;
                                        else
                                            point <= point + 1;
                                    end
                                else
                                    point <= point + 1;
                            end
                        end
                    // for infinite mode and countdown mode, one correct for one point
                    else
                        begin
                            if (theN[3:0] == playerN[3:0]) begin
                                if (theL[2:1] == 2'd2)
                                    begin
                                        if (ti <= 8'd2)
                                            point <= point + 3;
                                        else if (ti <= 8'd5)
                                            point <= point + 2;
                                        else
                                            point <= point + 1;
                                    end
                                else
                                    point <= point + 1;
                            end
                        end
                    // reset time and record number of random number in eight number mode
                    if (theL[2:1] == 2'd2) begin
                        reS = 0;
                        nRN <= nRN + 1;
                    end
                end
            if (display_input)
                begin
                    if (theL[0])
                        begin
                            numP1 <= playerN[7:4];
                            numP0 <= playerN[3:0];
                        end
                    else
                        begin
                            numP1 <= 0;
                            numP0 <= playerN[3:0];
                        end
                end
            if (loadRNum) // load random number
                begin
                    theN = randomN;
                end
            if (displayRNum) // display random number
                begin
                    if (theL[0])
                        begin
                            numR1 <= theN[7:4];
                            numR0 <= theN[3:0];
                        end
                    else
                        begin
                            numR1 <= 0;
                            numR0 <= theN[3:0];
                        end
                end
            if (loadPNum) // load player input
                begin
                    playerN <= data_in;
                end
        end else // for scoreboard
            begin
                if (displayRNum) // check for mode we want to see
                    begin
                        if (theL[2])
                            begin
                                if (theL[1])
                                    begin
                                        numP1 = 4'd3;
                                    end
                                else
                                    begin
                                        numP1 = 4'd2;
                                    end
                            end
                        else
                            begin
                                if (theL[1])
                                    begin
                                        numP1 = 4'd1;
                                    end
                                else
                                    begin
                                        numP1 = 4'd0;
                                    end
                            end
                        numP0 = theL[0]; // check for difficulty
                        // if it is easy countdown mode, read the corresponding record
                        if (theL[2:0] == 3'b010) begin
                            case (point)
                                8'd0: index_010 = 8'd0;
                                8'd1: index_010 = 8'd1;
                                8'd2: index_010 = 8'd2;
                                8'd3: index_010 = 8'd3;
                                8'd4: index_010 = 8'd4;
                                default: index_010 = 8'd0;
                            endcase
                            r_or_w_010 = 0;
                            // manually click to read
                            clk_010 = 1;
                            clk_010 = 0;
                            numR1 <= data_out_010[7:4];
                            numR0 <= data_out_010[3:0];
                        end
                    end

                if (display_input) // check the rank for displaying score
                    begin
                        point = point + 1;
                        if (point == 8'd5) // we only show top five score. So the next score will show after the fifth score would be the first one
                            point = 0;
                    end
            end
    end
endmodule

module clock(
    input clk,
    input reset,
    input enable,

    output reg [7:0] ti);
    
    reg [25:0] counter = 0;
    
    always@(posedge clk) begin
        if (!reset) begin
            counter = 0;
            ti = 0;
        end
        if (enable) begin
            counter = counter + 1;
            if (counter == 26'd49999999) // add 1 to time after 1s
                ti <= ti + 1;
        end 
    end
    
endmodule

module hex_decoder(hex_digit, segments);
    input [3:0] hex_digit;
    output reg [6:0] segments;
   
    always @(*)
        case (hex_digit)
            4'h0: segments = 7'b100_0000;
            4'h1: segments = 7'b111_1001;
            4'h2: segments = 7'b010_0100;
            4'h3: segments = 7'b011_0000;
            4'h4: segments = 7'b001_1001;
            4'h5: segments = 7'b001_0010;
            4'h6: segments = 7'b000_0010;
            4'h7: segments = 7'b111_1000;
            4'h8: segments = 7'b000_0000;
            4'h9: segments = 7'b001_1000;
            4'hA: segments = 7'b000_1000;
            4'hB: segments = 7'b000_0011;
            4'hC: segments = 7'b100_0110;
            4'hD: segments = 7'b010_0001;
            4'hE: segments = 7'b000_0110;
            4'hF: segments = 7'b000_1110;   
            default: segments = 7'h7f;
        endcase
endmodule

module mem5(
    input clk,
    input r_or_w,
    input [7:0] index,
    input [7:0] data_in,
    output reg [7:0] data_out);

    reg [7:0] line4, line3, line2, line1, line0;

    always @(posedge clk)
        // if write, do a linear sort
        if (r_or_w)
            begin
                if (data_in > line0)
                    begin
                        line4 <= line3;
                        line3 <= line2;
                        line2 <= line1;
                        line1 <= line0;
                        line0 <= data_in;
                    end
                else if (data_in > line1)
                    begin 
                        line4 <= line3;
                        line3 <= line2;
                        line2 <= line1;
                        line1 <= data_in;
                    end
                else if (data_in > line2)
                    begin 
                        line4 <= line3;
                        line3 <= line2;
                        line2 <= data_in;
                    end
                else if (data_in > line3)
                    begin 
                        line4 <= line3;
                        line3 <= data_in;
                    end
                else if (data_in > line4)
                    begin 
                        line4 <= data_in;
                    end
            end
        else // read based on index
            case (index)
                8'd0: data_out <= line0;
                8'd1: data_out <= line1;
                8'd2: data_out <= line2;
                8'd3: data_out <= line3;
                8'd4: data_out <= line4;
            endcase
endmodule

                
