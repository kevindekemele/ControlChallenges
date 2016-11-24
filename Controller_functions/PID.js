var cx = 0, // Setpoint
      i_e = 0; // Initial value of integral action

var P = 2,  // Proportional constant
      I = 0.0001, // Integral constant 
      D = 1.5 // Derivative constant

// Saturation
// Both bounds 0 = no saturation
var upper_bound = 0, // Upper bound controller
    lower_bound = 0; // Lower bound controller

var anti_reset_windup = false;

var prev_t = 0, // Previous time (for D and I)
      prev_p_e = 0; // Previous error (For D)


	  
function controlFunction(block) // Block is the model
{
  var p_e = cx - block.x, // Error, setpoint - measured
       d_e;					// derivative
  if(block.T !== 0){
    d_e = (p_e - prev_p_e)/((block.T - prev_t)); // Simple derivative
  }else{
    d_e = 0; // Initially, no derivative
  }

  i_e = i_e + (block.T-prev_t)*p_e; // Integral term


  prev_t = block.T;
  prev_p_e = p_e;

  var control = P * p_e + I * i_e + D*d_e; // Parallal PID

  // Anti reset_windup

if(!(upper_bound ==0 && lower_bound ==0)&& I !=0 && anti_reset_windup){
  if(control > upper_bound){
    control = upper_bound;
  }
  elseif(control < lower_bound){
    control = lower bound;
  }
  ie = (control - (P * p_e  + D*d_e))/I;  // Clipped, so integral term needs to be adjusted
}
  return control;
}