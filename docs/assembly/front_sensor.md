# Front Sensor Array

This subassembly contains the main, forward-facing sensors used by the car for object and environmental detection.

![asm](img/asm/front_sensor/asm.svg)

## I. Requirements
### Tools

|        | Description| 
|:------:|:-----------|
|![screwdriver](img/tools/driver.svg)| 2.0mm ![hex](img/tools/bits/hex_inner.svg) Hex Driver, <br> 2.5mm ![hex](img/tools/bits/hex_inner.svg) Hex Driver,<br> 3.0mm ![hex](img/tools/bits/hex_inner.svg) Hex Driver,<br> 3/8" ![hex](img/tools/bits/hex_inner.svg) Hex Driver     |
|![iron](img/tools/soldering_iron.svg)| Soldering Iron     |
|![glue](img/tools/glue.svg)| Cyanoacrilate, Two-part Epoxy or Silicon Glue     |


### Parts

#### Hardware

|                                                  | Description                | Qty |   |                                                  | Description                | Qty |
|:------------------------------------------------:|:---------------------------|:---:|---|:------------------------------------------------:|:---------------------------|:---:|
|![scr_cbr_m2.5x16](img/hw/scr/cbr/m2.5x16.svg)    |M2.5x16 CBR Screws          | 2   |   |![nut_hex____m3](img/hw/nut/hex/m3.svg)           |M3 Nuts                     | 4   |
|![scr_cbr_m3x06](img/hw/scr/csk/m3x08.svg)        |M3x8 CSK Screws             | 16  |   |![nut_loc____m3](img/hw/nut/loc/m3.svg)           |M3 Nyloc Nuts               | 2   |
|![scr_csk_m3x10](img/hw/scr/csk/m3x10.svg)        |M3x10 CSK Screws            | 2   |   |![nut_hex____m4](img/hw/nut/hex/m4.svg)           |M4 Nuts                     | 4   |
|![scr_csk_m3x08](img/hw/scr/csk/m3x12.svg)        |M3x12 CSK Screws            | 2   |   |![nut_loc____m4](img/hw/nut/loc/m4.svg)           |M4 Nuts                     | 4   |
|![scr_csk_m4x12](img/hw/scr/cbr/m4x10.svg)        |M4x10 CBR Screws            | 21  |   |![nut_tee____m4](img/hw/nut/tee/m4.svg)           |M4x6 T-Nuts                 | 8   |
|![scr_cbr__1/2"](img/hw/scr/cbr/in0.25-20x0.5.svg)|1/4"-20x1/2" CBR Screws     | 3   |   |                                                  |                            |     |






#### Custom Parts

|                                                            | Description          | Qty |   |                                                       | Description                | Qty |
|:----------------------------------------------------------:|:---------------------|:---:|---|:-----------------------------------------------------:|:---------------------------|:---:|
|![01](img/asm/front_sensor/prt/01_vertical_rail.svg)        | Sensor Vertical Rail | 2   |   |![06](img/asm/front_sensor/prt/06_hand_knob.svg)       | Hand Knob                  | 2   |
|![02](img/asm/front_sensor/prt/02_rail_horizontal_brace.svg)| Rail Horizontal Brace| 2   |   |![07](img/asm/front_sensor/prt/08_sick_bracket.svg)    | Sick Bracket               | 1   |
|![03](img/asm/front_sensor/prt/03_left_brace.svg)           | Left Brace           | 1   |   |![10](img/asm/front_sensor/prt/10_camera_bracket.svg)  | Camera Bracket             | 2   |
|![04](img/asm/front_sensor/prt/04_right_brace.svg)          | Right Brace          | 1   |   |![11](img/asm/front_sensor/prt/11_cam_back_support.svg)| Camera Bracket Back Support| 2   |
|![05](img/asm/front_sensor/prt/05_brace_nut.svg)            | Brace Nut            | 2   |   |![14](img/asm/front_sensor/prt/14_vive_bracket.svg)    | Vive Bracket               | 1   |



#### Commercial Components
|                                                    | Description                        | Qty |
|:--------------------------------------------------:|:-----------------------------------|:---:|
|![01](img/asm/front_sensor/prt/09_sick_tim55x.svg)  | Sick TIM55x Ranging Laser Scanner  | 1   |
|![02](img/asm/front_sensor/prt/12_zed_cam.svg)      | ZED Stereo camera                  | 1   |
|![03](img/asm/front_sensor/prt/13_intel_d435i.svg)  | Intel RealSense Depth Camera D435i | 1   |
|![04](img/asm/front_sensor/prt/15_vive_tracker.svg) | HTC Vive Tracker                   | 1   |

## II. Assembly Steps

### II.1 Array Screws

1. Insert one *M4 Nut* on one end of the *M4x270* and use another *M4 Nut* to jam it in place so as to prevent them moving. Up to 5mm of thread can be left after the nuts. Repeat this step for the second screw.  

    ![s0101](img/asm/front_sensor/stp/S0101.svg)
   
 2. Attach the ***Hand Knob*** to the threaded rod using some glue, preferrably two-part epoxy, but silicone or cyanoacrylate based ones should work as well. Make sure that the hexagonal faces of the outermost nut engage properly with those in the *Hand Knob* and that it is seated as deeply as possible. Repeat this step for the second screw.

    ![s0101](img/asm/front_sensor/stp/S0102.svg)
   
 3. Final assembly. Let the glue fully cure according to the manufacturer's instructions before applying any forces to the screws.

    ![s0101](img/asm/front_sensor/stp/S0103.svg)

### II.2 Mounting Bracket

1. Insert two *M4 Nyloc Nuts* into the respective holes as shown in the picture. Use some form of glue to preventing from falling out when attaching or removing the screws.

    ![s0201](img/asm/front_sensor/stp/S0201.svg)
   
 2. Insert two *M3 Nyloc Nuts* into the respective holes as shown in the picture. Optionally, use glue to affix them in place, though it is less likely that they will fall out.

    ![s0202](img/asm/front_sensor/stp/S0202.svg)
   
 3. Insert the GR8LE's ***Body Columns*** into their respective square bosses. Screw them in place using the car's original *M2.5x16 CBR* screws using the bottom holes from the columns. 

    ![s0203](img/asm/front_sensor/stp/S0203.svg)
    
 4. Final assembly. Let the glue fully cure according to the manufacturer's instructions before applying any forces to the assembly.

    ![s0204](img/asm/front_sensor/stp/S0204.svg)


### II.3 Vertical Rails

 1. Using a soldering iron, place the *M3 Brass Threaded Inserts* into the ***Sensor Vertical Rail***, leaving it flush with the top surface. Repeat for the second rail. 

    ![s0301](img/asm/front_sensor/stp/S0301.svg)
   
 2. Fasten the ***Horizontal Rail Braces*** to one of the ***Vertical Sensor Rails*** using four *M4x10 Socket Head* screws.

    ![s0302](img/asm/front_sensor/stp/S0302.svg)
   
 3. Using another four *M4x10 Socket Head* screws, fasten the second ***Vertical Sensor Rail*** to the previous subassembly.

    ![s0303](img/asm/front_sensor/stp/S0303.svg)
    
 4. Attach the ***Left*** and ***Right*** Braces to the subassembly using two *M3x8 Countersunk* screws.

    ![s0304](img/asm/front_sensor/stp/S0304.svg)
 
 5. Insert the ***Brace Nuts*** into the ***Braces***, making sure that the hexagonal faces are aligned, and secure them using two *M3x12 Countersunk* screws. Leave a gap between the bottom face of the brace and the flange of the nut of about 3.5mm for an easier installation on the ***PC Case*** later. 

    ![s0305](img/asm/front_sensor/stp/S0305.svg)
    
 6. Final assembly.

    ![s0306](img/asm/front_sensor/stp/S0306.svg)

### II.4 Vive Tracker Bracket

 1. Attach two ***M4x10 Socket Head Screws*** and two ***M4x6 Tee Nuts*** to the ***Vive Tracker Bracket***, leaving them loose for easy insertion and removal from the ***Rail*** Subassembly.

    ![s0401](img/asm/front_sensor/stp/S0401.svg)
   
 2. Screw an ***M4x10 Socket Head Screw*** into any of the six holes of the ***Vive Tracker Bracket***.
 The purpose of this screw is to prevent the rotation of the ***Tracker***, and its position determines the angular orientation of the tracker with respect to the rest of the car.
 The recommended position shown in the picture is so that the *z* axis of the ***Tracker*** is pointing towards the front of the car.

    ![s0402](img/asm/front_sensor/stp/S0402.svg)
   
 3. Secure the ***Vive Tracker*** to the ***Bracket*** using a *1/2"-20x0.500" Socket Head* Screw, making sure to also align the screw from the previous step with the *Stabilizing Pin Recess** on the bottom of the ***Tracker***.

    ![s0403](img/asm/front_sensor/stp/S0403.svg)
    
 4. Final assembly.

    ![s0404](img/asm/front_sensor/stp/S0404.svg)

### II.5 Intel Camera Bracket

 1. Attach two ***M4x10 Socket Head Screws*** and two ***M4x6 Tee Nuts*** to the ***Camera Bracket***, leaving them loose for easy insertion and removal from the ***Rail*** Subassembly.

    ![s0501](img/asm/front_sensor/stp/S0501.svg)
   
 2. Mount the ***Camera Bracket Back Support*** to the ***Camera Bracket's*** front holes using two *M3x8 Countersunk Screws*.

    ![s0502](img/asm/front_sensor/stp/S0502.svg)
   
 3. Secure the ***Intel RealSense Camera*** to the ***Bracket*** using a *1/2"-20x0.500" Socket Head* Screw.

    ![s0503](img/asm/front_sensor/stp/S0503.svg)
    
 4. Optionally, secure the ***Camera*** to the ***Back Support*** using two *M3x8 Countersunk* screws.

    ![s0504](img/asm/front_sensor/stp/S0504.svg)
 
 5. Final assembly.

    ![s0505](img/asm/front_sensor/stp/S0505.svg)
    
### II.6 ZED Camera Bracket

 1. Attach two ***M4x10 Socket Head Screws*** and two ***M4x6 Tee Nuts*** to the ***Camera Bracket***, leaving them loose for easy insertion and removal from the ***Rail*** Subassembly.

    ![s0601](img/asm/front_sensor/stp/S0501.svg)
   
 2. Mount the ***Camera Bracket Back Support*** to the ***Camera Bracket's*** rear holes using two *M3x8 Countersunk Screws*.

    ![s0602](img/asm/front_sensor/stp/S0602.svg)
   
 3. Secure the ***ZED Camera*** to the ***Bracket*** using a *1/2"-20x0.500" Socket Head* Screw.

    ![s0603](img/asm/front_sensor/stp/S0603.svg)
    
 4. Final assembly.

    ![s0604](img/asm/front_sensor/stp/S0604.svg)

### II.7 SICK Scanner Bracket

 1. Attach two ***M4x10 Socket Head Screws*** and two ***M4x6 Tee Nuts*** to the ***SICK Bracket***, leaving them loose for easy insertion and removal from the ***Rail*** Subassembly.

    ![s0701](img/asm/front_sensor/stp/S0701.svg)
   
 2. Mount the ***Sick Ranging Laser Scanner*** to the ***Bracket*** using two *M3x8 Countersunk* screws. 

    ![s0702](img/asm/front_sensor/stp/S0702.svg)
   
 3. Final Assembly.

    ![s0703](img/asm/front_sensor/stp/S0703.svg)
    
### II.8 Final Assembly

1. Slide each of the sensor brackets into their desired position, making sure that the *Tee Nuts* are engaging properly with the T-Slots. Once satisfied with their positioning, tighten all of the *M4x8 Socket Head* screws holding them in place.

    ![s0801](img/asm/front_sensor/stp/S0801.svg)
   
 2. Final –proposed– assembly.

    ![s0802](img/asm/front_sensor/stp/S0802.svg)



## III. Exploded View

### III.1 Mounting Bracket
![exp](img/asm/front_sensor/exp_bracket.svg)

#### BOM

| ID     | Description                     | Qty |
|:------:|:--------------------------------|:---:|
|  **1** |Front Sensor Bracket             | 1   |
|  **2** |Body Columns                     | 2   |
|  **3** |M2.5x16 CBR Screws               | 2   |
|  **4** |M3x10 CSK Screws                 | 2   |
|  **5** |M3 Nyloc Nuts                    | 2   |
|  **6** |M4 Nyloc Nuts                    | 2   |






### III.2 Sensor Array

![exp](img/asm/front_sensor/exp.svg)

#### BOM

| ID     | Description                     | Qty |     | ID     | Description                      | Qty |
|:------:|:--------------------------------|:---:|:---:|:------:|:--------------------------------:|:---:|
|  **1** |Sensor Vertical Rail             | 2   |     | **13** |Intel RealSense Depth Camera D435i| 1   |
|  **2** |Horizontal Rail Brace            | 2   |     | **14** |Vive Tracker Bracket              | 1   |
|  **3** |Left Brace                       | 1   |     | **15** |HTC Vive Tracker                  | 1   |
|  **4** |Right Brace                      | 1   |     | **16** |M3x8 CSK Screws                   | 16  |
|  **5** |Brace Nut                        | 2   |     | **17** |M3x12 CSK Screws                  | 2   |
|  **6** |Hand Knob                        | 2   |     | **18** |M4x10 CBR Screws                  | 21  |
|  **7** |M3x270 Threaded Rod              | 2   |     | **19** |1/4"-20x1/2" CBR Screws           | 3   |
|  **8** |Sick Bracket                     | 1   |     | **20** |M3 Nuts                           | 4   |
|  **9** |Sick TIM55x Ranging Laser Scanner| 1   |     | **21** |M4 Nuts                           | 4   |
| **10** |Camera Bracket                   | 2   |     | **22** |M4x6 Tee Nuts                     | 8   |
| **11** |Camera Bracket Back Support      | 2   |     | **23** |M4 Brass Threaded Insert          | 2   |
| **12** |ZED Stereo Camera                | 1   |









