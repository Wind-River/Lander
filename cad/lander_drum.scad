// Metric Screw Thread Library
// by Maximilian Karl <karlma@in.tum.de> (2012)
// 
//
// only use module thread(P,D,h,step)
// with the parameters:
// P    - screw thread pitch
// D    - screw thread major diameter
// h    - screw thread height
// step - step size in degree
// 
include <MCAD\polyholes.scad>

FlatShaft=true;


module screwthread_triangle(P) {
	difference() {
		translate([-sqrt(3)/3*P+sqrt(3)/2*P/8,0,0])
		rotate([90,0,0])
		cylinder(r=sqrt(3)/3*P,h=0.00001,$fn=3,center=true);

		translate([0,-P/2,-P/2])
		cube([P,P,P]);
	}
}

module screwthread_onerotation(P,D_maj,step) {
	H = sqrt(3)/2*P;
	D_min = D_maj - 5*sqrt(3)/8*P;

	for(i=[0:step:360-step])
	hull()
		for(j = [0,step])
		rotate([0,0,(i+j)])
		translate([D_maj/2,0,(i+j)/360*P])
		screwthread_triangle(P);

	translate([0,0,P/2])
	cylinder(r=D_min/2,h=2*P,$fn=360/step,center=true);
}

module thread(P,D,h,step) {
	for(i=[0:h/P])
	translate([0,0,i*P])
	screwthread_onerotation(P,D,step);
}

difference()
{
	difference()
	{
		union()
		{
			// example
			thread(0.8,20.0,12,10);
			// P    - screw thread pitch
			// D    - screw thread major diameter
			// h    - screw thread height
			// step - step size in degree
			
			translate([0,0,-0.4])
			{
				cylinder(r=13.5,h=1);
			}
			translate([0,0,12])
			{
				cylinder(r=13.5,h=5);
			}
		}
		
		translate([0,0,-0.5])
		{
			difference()
			{
				cylinder(r=2.85, h=18, $fn=100);
				//polyhole(18,5);
				if(FlatShaft)
				{
					translate([-2.5,2,-1])
					{
						cube([5,5,22]);
					}
				}
			}
		}
	}
	if(FlatShaft)
	{
		translate([0,0,14.4])
		{
			rotate([-90,90.0])
			{
				cylinder(r=1.25,h=14, $fn=100);
				//polyhole(14,2.5);
			}
		}
	}
	else
	{
		for(i = [0:120:360])
		{
			rotate([0,0,i])
			{
				translate([0,0,14.4])
				{
					rotate([-90,90.0])
					{
						cylinder(r=1.25,h=14, $fn=100);
						//polyhole(14,2.5);
					}
				}
			}
		}
	}
}