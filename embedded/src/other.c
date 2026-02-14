

/*
joystick pinout:
    -gnd + vcc (gnd and 3v)
    -VRx - left and right (left is 0v, right is 3.3)
    -VRy - up and down    (up is 5, down is 0)          -middle for both is like 1.5ish
    -SW  - button in middle (unused)

PA1 - left and right
PA2 - amount of thrust (sliding resistor)
*/



// get data from joystick and return it in a range
// probs y = 1023-767 is 2 right,
//           767 -520 is 1 right (give some leway so bumping it doesnt affect it)
//           500 -255 is 1 left
//           255 - 0  is 2 left
// not exactly "2" or "1", just make it move more if you go farther?



// //make a function to update the movement variables 
// void update_movement_variables(){
//     /*
//     get data from joystick -> returns in a range 1023 - 0

//     probs y = 1023-767 is 2 right,
//               767 -520 is 1 right (give some leway so bumping it doesnt affect it)
//               500 -255 is 1 left
//               255 - 0  is 2 left
//     not exactly "2" or "1", just make it move more if you go farther?
//     */
//    //im gpoing to the ece shop todau to get a joystick and i can see what values it returns instead of just seeing datasheet
//     // fuel -= thrust_down; 
//    fuel -= thrust_side;

//    //check if fuel
//    if (fuel <= 0) {thrust_side = 0; fuel = 0;}


//     //general movement unrelated to the joystick
//     velo_side += thrust_side; //or something idk but this would need negative values


//     // alt += velo_down;
//     // if(alt <= 0){    //       check if `-velo` < 10
//     //     if(-velo_down < 10) {mode = 'L'; return;}
//     // //else:
//     //     mode = 'C'; return;
//     // }

//     // velo_down += (thrust - 5);
// }
