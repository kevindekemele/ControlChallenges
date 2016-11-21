'use strict';
if (typeof Models === 'undefined') var Models = {};

Models.SinglePendulum = function(params)
{
	var nVars = Object.keys(this.vars).length;
	for(var i = 0; i < nVars; i++)
	{
		var key = Object.keys(this.vars)[i];
		this[key] = (typeof params[key] == 'undefined')?this.vars[key]:params[key];
	}
}

Models.SinglePendulum.prototype.vars = 
{
	m0: 10,
	m1: .5,
	L: 1,
	g: 9.81,
	theta: 0,
	dtheta: 0,
	x: 1,
	dx: 0,
	F: 0,
	T: 0,
    d: 0
};

Models.SinglePendulum.prototype.simulate = function (dt, controlFunc)
{
	var copy = new Models.SinglePendulum(this);
	var state = [this.theta, this.dtheta, this.x, this.dx];
	copy.F = controlFunc(new Models.SinglePendulum(this));
	//copy.F = Math.max(-50,Math.min(50,copy.F));
	if(typeof copy.F != 'number' || isNaN(copy.F)) throw "Error: The controlFunction must return a number.";
	var soln = numeric.dopri(0,dt,state,function(t,x){ return Models.SinglePendulum.ode(copy,x); },1e-10).at(dt);	
	
	copy.x = soln[2];
	copy.dx = soln[3];
	copy.theta = soln[0];
	copy.dtheta = soln[1];
	copy.T = this.T + dt;
	return copy;
}

Models.SinglePendulum.ode = function (_this, x)
{
	var s = Math.sin(x[0]);
	var c = Math.cos(x[0]);
	var dthetasq = x[1] * x[1];
	var f = _this.F+_this.d;
	var ms = _this.m1;
	var mb = _this.m0;
	var g = _this.g;
	var L = _this.L;
	
	var num2 = mb + ms - ms*c*c;
	var num1 = num2*L;
	var M = [[_this.m0,0,0,0,-s],
		[0,0,_this.m1,0,s],
		[0,0,0,_this.m1,c],
		[1,_this.L*c,-1,0,0],
		[0,-_this.L*s,0,-1,0]];
	var b = [_this.F,0,-_this.m1*_this.g,s*dthetasq*_this.L,c*dthetasq*_this.L];
	var ddx = numeric.solve(M,b)
	return [x[1],((ms+mb)*g*s-c*f-ms*L*s*c*dthetasq)/num1,x[3],(f+ms*L*s*dthetasq-ms*c*s*g)/num2];
}


Models.SinglePendulum.prototype.draw = function (ctx, canvas)
{
	resetCanvas(ctx,canvas);
	ctx.translate(0,-this.L);
	
	var cartWidth = 0.4*this.L;
	var cartHeight = 0.7*cartWidth;
	
	
	var tipX = ((this.x+4)%8)-4+this.L*Math.sin(this.theta);
	var tipY = this.L*Math.cos(this.theta)+cartHeight;
	
	// ground
	ctx.strokeStyle="#333366";
	drawLine(ctx,-100,-.025,100,-.025,0.05);
	
	// cart
	ctx.fillStyle="#4444FF";
	ctx.fillRect(((this.x+4)%8)-4-cartWidth/2,0,cartWidth,cartHeight);
		
	// shaft
	ctx.strokeStyle="#AAAAFF";
    ctx.lineCap = 'round';
	drawLine(ctx,((this.x+4)%8)-4,cartHeight,tipX,tipY,this.L/20.0);
		
	// tip-mass
	ctx.beginPath();
	ctx.arc(tipX, tipY, this.L/7, 0, 2 * Math.PI, false);
	ctx.fillStyle = '#4444FF';
	ctx.fill();
	
	// force arrow
	var forceArrow = {x1:((this.x+4)%8)-4,y1:0.5*cartHeight,x2:((this.x+4)%8)-4+0.1*this.F,y2:0.5*cartHeight};
	ctx.strokeStyle="#FF0000";
    ctx.lineCap = 'round';	
	drawLine(ctx,forceArrow.x1,forceArrow.y1,forceArrow.x2,forceArrow.y2,this.L/40.0);
	drawLine(ctx,forceArrow.x2,forceArrow.y2,forceArrow.x2-Math.sign(this.F)*0.1,forceArrow.y2+0.05,this.L/40.0);
	drawLine(ctx,forceArrow.x2,forceArrow.y2,forceArrow.x2-Math.sign(this.F)*0.1,forceArrow.y2-0.05,this.L/40.0);
}

Models.SinglePendulum.prototype.infoText = function ()
{
	return  "/* Horizontal position       */ pendulum.x      = " + round(this.x,2)
		+ "\n/* Horizontal velocity       */ pendulum.dx     = " + round(this.dx,2)
		+ "\n/* Angle from vertical (rad) */ pendulum.theta  = " + round(this.theta,5)
		+ "\n/* Angular velocity (rad/s)  */ pendulum.dtheta = " + round(this.dtheta,2)
		+ "\n/* Simulation time (s)       */ pendulum.T      = " + round(this.T,2)
        + "\n/* Control effort (N)       */ pendulum.F      = " + round(this.F,2)
        + "\n/* Distrbance      (N) */ pendulum.d      = " + round(this.d,2);
		
}
