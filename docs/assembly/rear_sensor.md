# Rear Sensor Array

This subassembly contains the rear-facing sensors used by the car for localization or other tasks.

![asm](img/asm/rear_sensor/asm.svg)

## I. Requirements
### Tools

|        | Description| 
|:------:|:-----------|
|![screwdriver](img/tools/driver.svg)| 2.0mm ![hex](img/tools/bits/hex_inner.svg) Hex Driver, <br> 2.5mm ![hex](img/tools/bits/hex_inner.svg) Hex Driver,<br> 3.0mm ![hex](img/tools/bits/hex_inner.svg) Hex Driver     |
|![glue](img/tools/glue.svg)| Cyanoacrilate, Two-part Epoxy or Silicon Glue     |


### Parts

#### Hardware

|                                                  | Description                | Qty |   |                                                  | Description                | Qty |
|:------------------------------------------------:|:---------------------------|:---:|---|:------------------------------------------------:|:---------------------------|:---:|
|![scr_cbr_m2.5x16](img/hw/scr/cbr/m2.5x16.svg)    |M2.5x16 CBR Screws          | 2   |   |![nut_loc____m3](img/hw/nut/loc/m3.svg)           |M3 Nyloc Nuts               | 2   |
|![scr_cbr_m3x06](img/hw/scr/csk/m3x08.svg)        |M3x8 CSK Screws             | 2   |   |![nut_hex____m4](img/hw/nut/hex/m4.svg)           |M4 Nuts                     | 4   |
|![scr_csk_m3x16](img/hw/scr/csk/m3x16.svg)        |M3x16 CSK Screws            | 2   |   |![nut_loc____m4](img/hw/nut/loc/m4.svg)           |M4 Nyloc Nuts               | 2   |
|![scr_csk_m4x12](img/hw/scr/cbr/m4x10.svg)        |M4x10 CBR Screws            | 6   |   |![nut_tee____m4](img/hw/nut/tee/m4.svg)           |M4x6 T-Nuts                 | 2   |


#### Custom Parts

|                                                            | Description          | Qty |   |                                                       | Description                | Qty |
|:----------------------------------------------------------:|:---------------------|:---:|---|:-----------------------------------------------------:|:---------------------------|:---:|
|![01](img/asm/rear_sensor/prt/01_vertical_rail.svg)         | Sensor Vertical Rail | 2   |   |![06](img/asm/front_sensor/prt/06_hand_knob.svg)       | Hand Knob                  | 2   |
|![02](img/asm/front_sensor/prt/02_rail_horizontal_brace.svg)| Rail Horizontal Brace| 2   |   |![07](img/asm/rear_sensor/prt/02_angle_cam_bracket.svg)| Tracking Camera Bracket    | 1   |



#### Commercial Components
|                                                 | Description                          | Qty |
|:-----------------------------------------------:|:-------------------------------------|:---:|
|![01](img/asm/rear_sensor/prt/03_intel_t265.svg) | Intel RealSense Tracking Camera T265 | 1   |

## II. Assembly Steps

### II.1 Array Screws

1. Insert one *M4 Nut* on one end of the *M4x135* and use another *M4 Nut* to jam it in place so as to prevent them moving. Up to 5mm of thread can be left after the nuts. Repeat this step for the second screw.  

    ![s0101](img/asm/rear_sensor/stp/S0101.svg)
   
 2. Attach the ***Hand Knob*** to the threaded rod using some glue, preferrably two-part epoxy, but silicone or cyanoacrylate based ones should work as well. Make sure that the hexagonal faces of the outermost nut engage properly with those in the ***Hand Knob*** and that it is seated as deeply as possible. Repeat this step for the second screw.

    ![s0101](img/asm/rear_sensor/stp/S0102.svg)
   
 3. Final assembly. Let the glue fully cure according to the glue manufacturer's instructions before applying any forces to the screws.

    ![s0101](img/asm/rear_sensor/stp/S0103.svg)

### II.2 Mounting Bracket

1. Insert two *M4 Nyloc Nuts* into the respective holes as shown in the picture. Use some form of glue to preventing from falling out when attaching or removing the screws.

    ![s0201](img/asm/rear_sensor/stp/S0201.svg)
   
 2. Insert two *M3 Nyloc Nuts* into the respective holes as shown in the picture. Optionally, use glue to affix them in place, though it is less likely that they will fall out.

    ![s0202](img/asm/rear_sensor/stp/S0202.svg)
   
 3. Insert the GR8LE's ***Rear Body Columns*** into their respective square bosses. Screw them in place using the car's original *M2.5x16 CBR* screws using the bottom holes from the columns. 

    ![s0203](img/asm/rear_sensor/stp/S0203.svg)
    
 4. Final assembly. Let the glue fully cure according to the glue manufacturer's instructions before applying any forces to the assembly.

    ![s0204](img/asm/rear_sensor/stp/S0204.svg)


### II.3 Vertical Rails

 1. Fasten the ***Horizontal Rail Brace*** to one of the ***Rear Vertical Sensor Rails*** using two *M4x10 Socket Head* screws.

    ![s0301](img/asm/rear_sensor/stp/S0301.svg)
   
 2. Using another two *M4x10 Socket Head* screws, fasten the second ***Rear Vertical Sensor Rail*** to the previous subassembly.

    ![s0302](img/asm/rear_sensor/stp/S0302.svg)
   
 3. Final assembly.

    ![s0303](img/asm/rear_sensor/stp/S0303.svg)

### II.4 Intel Tracking Camera Bracket

 1. Attach two ***M4x10 Socket Head Screws*** and two ***M4x6 Tee Nuts*** to the ***Intel Tracking Camera Bracket***, leaving them loose for easy insertion and removal from the ***Rail*** Subassembly.

    ![s0401](img/asm/rear_sensor/stp/S0401.svg)
   
 2. Mount the ***Intel RealSense Tracking Camera T265*** to the ***Bracket*** using two *M3x8 Countersunk* screws.

    ![s0402](img/asm/rear_sensor/stp/S0402.svg)
   
 3. Final assembly.

    ![s0403](img/asm/rear_sensor/stp/S0403.svg)
    

### II.5 Final Assembly

 1. Slide each of the sensor brackets into their desired position, making sure that the *Tee Nuts* are engaging properly with the T-Slots. Once satisfied with their positioning, tighten all of the *M4x8 Socket Head* screws holding them in place.

    ![s0501](img/asm/rear_sensor/stp/S0501.svg)
   
 2. Final –proposed– assembly.

    ![s0502](img/asm/rear_sensor/stp/S0502.svg)



## III. Exploded View

### III.1 Mounting Bracket
![exp](img/asm/rear_sensor/exp_bracket.svg)

#### BOM

| ID     | Description                     | Qty |
|:------:|:--------------------------------|:---:|
|  **1** |Rear Sensor Bracket              | 1   |
|  **2** |Rear Body Columns                | 2   |
|  **3** |M2.5x16 CBR Screws               | 2   |
|  **4** |M3x16 CSK Screws                 | 2   |
|  **5** |M3 Nyloc Nuts                    | 2   |
|  **6** |M4 Nyloc Nuts                    | 2   |






### III.2 Sensor Array

![exp](img/asm/rear_sensor/exp.svg)

#### BOM

| ID     | Description                         | Qty |     | ID     | Description                      | Qty |
|:------:|:------------------------------------|:---:|:---:|:------:|:--------------------------------:|:---:|
|  **1** |Rear Sensor Vertical Rail            | 2   |     | **7**  |M3x8 CSK Screws                   | 2   |
|  **2** |Horizontal Rail Brace                | 1   |     | **8**  |M4x10 CBR Screws                  | 2   |
|  **3** |Hand Knob                            | 2   |     | **9**  |M4x135 Threaded Rod               | 2   |
|  **4** |Tracking Camera Bracket              | 1   |     | **10** |M4 Nuts                           | 4   |
|  **5** |Intel RealSense Tracking Camera T265 | 1   |     | **11** |M4x6 Tee Nuts                     | 2   |
|  **6** |M4x10 CBR Screws                     | 4   |     |        |                                  |     |









