Verilog Tetris
=============

We made Tetris using an FPGA and Verilog. You can find our report in this repository for more information. Here are the basics:

#Introduction:
+ For our final project we wanted to implement the famous game known as Tetris. The game interfaces with the VGA monitor and uses on board buttons and such. We build this entire thing from scratch except for the debouncer and VGA example that we build off of. Unfortunately we had some problems and were only able to finish some of the game pieces as we ran out of resources on the FPGA board.

#Description:
+ In the game of Tetris the goal is to clear as many lines as possible. To do this the player must move and rotate random pieces that fall from the sky one at a time to create full horizontal lines on the board, clearing them. Each piece is made out of four Tetriminos. These pieces can be rotated and moved across the screen as needed. When a row is cleared the user gains one point. The blocks must stay within the allotted grid. The user can click and hold left and right to move multiple steps or click and hold down to teleport the piece to the bottom.

#How to Play:
+ Connect the VGA to the board and make sure all the switches are set to zero. After this is loaded you toggle switch two up and down to start the game. Clicking the left button moves the block to the left. Clicking the right button moves the block to the right. Clicking up rotates the brick. Clicking and holding down will teleport the brick down. If you lose toggle the start switch up and down to restart the game. The score is recorded on the LED display

#Disclaimer:
+ We are still working on adding all of the bricks to make the game more like the real tetris
