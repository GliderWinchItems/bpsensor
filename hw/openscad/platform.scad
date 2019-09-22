/* File: platform.scad
 * Platform to hold sensor perfboard and RJ jack sub board
 * Author: deh
 * Latest edit: 20190726
 */
 
 $fn = 25;
 
 hvthick = 4;
 
 hvbase = [93,70,hvthick];

holehv1 = [   3,    7,  0];
holehv2 = [55.5,    7,  0];
holehv3 = [55.0, 62.5,  0];
holehv4 = [   3, 62.5,  0];
holehv5 = [   3, 62.5,  0];
holehv6 = [86.0, 10.8,  0];
holehv7 = [86.0, 20.5,  0];
holehv8 = [88.0, 37.5,  0];



module hvpost(a)
 {
     translate(a)
     {
         difference()
        {
            cylinder(d1=9.0, d2=5.5,h=(4+hvthick),center=false);
            
            translate([0,0,(4+hvthick)-3.5])
            cylinder(d1=1.0, d2=2.8,h=3.5,center=false);
        }
     }
     
 }
 
 module hvbase()
 {
     difference()
     {
		  union()
   	  {
            hvpost(holehv1);
            hvpost(holehv2);
            hvpost(holehv3);
            hvpost(holehv4);
            hvpost(holehv5);
            hvpost(holehv6);
            hvpost(holehv7);
            hvpost(holehv8);

            cube(hvbase,false);
        }   
        union()
        {
            translate([60,-.01,0])
                cube([35.5,7,hvthick+.01],false);

            translate([60,43,0])
                cube([34,50,hvthick+.01],false);
            
            translate([29,35,-0.1])
                cube([40,50,50],true);
            
            translate([70,26,0])
                cube([22,20,50],true);
            
        }
     }
 }
 
hvbase();
 
 // Tab to hold to platform1
 translate([-15,0,0])
    cube([15,70,1.5],false);
