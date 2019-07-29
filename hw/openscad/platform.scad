/* File: platform.scad
 * Platform to hold sensor perfboard and RJ jack sub board
 * Author: deh
 * Latest edit: 20190726
 */
 
 base = [92,70,2];
 
 module base()
 {
     difference()
     {
        cube(base,false);
         
        union()
        {
            translate(60,0,0);
                cube([33,17,0],false);

            translate(60,43,0);
                cube([33,50,0],false);
            
        }
     }
 }
 
base();
