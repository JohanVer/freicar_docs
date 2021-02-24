# PC Case

The purpose of this subassembly is to encase the major electronic components used for the FreiCar, provide protection from environmental hazards such as dust and moisture, and allow some degree of cable management.

![asm](img/asm/pc_case/asm.svg)

## I. Requirements
### Tools




### Parts

#### Hardware

|                                          | Description                | Qty | 
|:----------------------------------------:|:---------------------------|:---:|
|![scr_cbr_m3x06](img/hw/scr/cbr/m3x06.svg)|M3x6 CBR Screws             | 12  |
|![scr_csk_m3x08](img/hw/scr/csk/m3x08.svg)|M3x8 CSK Screws             | 12  |
|![scr_csk_m4x12](img/hw/scr/csk/m4x12.svg)|M4x12 CSK Screws            | 12  |

#### Custom Parts

|                                                    | Description         | Qty |   |                                                    | Description         | Qty |
|:--------------------------------------------------:|:--------------------|:---:|---|:--------------------------------------------------:|:--------------------|:---:|
|![01_bottom](img/asm/pc_case/prt/01_bottom.svg)     | Case Bottom         | 1   |   |![08_left](img/asm/pc_case/prt/08_left.svg)         | Case Left           | 1   |
|![02_c90](img/asm/pc_case/prt/02_corner_90.svg)     | Case Corner 90º     | 1   |   |![09_right](img/asm/pc_case/prt/09_right.svg)       | Case Right          | 1   |
|![03_cp](img/asm/pc_case/prt/03_corner_pci.svg)     | Case Corner PCI     | 1   |   |![10_f_left](img/asm/pc_case/prt/10_front_left.svg) | Case Front-Left     | 1   |
|![04_ca1](img/asm/pc_case/prt/04_corner_angle_1.svg)| Case Corner Angle 1 | 2   |   |![11_b_left](img/asm/pc_case/prt/11_back_left.svg)  | Case Back-Left      | 1   |
|![05_ca2](img/asm/pc_case/prt/05_corner_angle_2.svg)| Case Corner Angle 2 | 2   |   |![12_top](img/asm/pc_case/prt/12_top.svg)           | Case Top            | 1   |
|![06_front](img/asm/pc_case/prt/06_front.svg)       | Case Front          | 1   |   |![13_so_10](img/asm/pc_case/prt/13_standoff_10.svg) | Standoff 10 mm      | 8   |
|![07_back](img/asm/pc_case/prt/07_back.svg)         | Case Back           | 1   |   |![14_so_40](img/asm/pc_case/prt/14_standoff_40.svg) | Standoff 40 mm      | 4   |
|![20_if](img/asm/pc_case/prt/20_interface.svg)      | Car Interface Board | 1   |   |                                                    |                     |     |



#### Commercial Components
|                                                    | Description         | Qty |
|:--------------------------------------------------:|:--------------------|:---:|
|![15_switch](img/asm/pc_case/prt/15_switch.svg)     | Toggle Switch w/ LED| 2   |
|![16_jack](img/asm/pc_case/prt/16_power_jack.svg)   | DC Power Jack       | 1   |
|![17_pc](img/asm/pc_case/prt/17_pc.svg)             | Mini-ITX PC         | 1   |
|![18_pc_cover](img/asm/pc_case/prt/18_pc_cover.svg) | PC Port Plate       | 1   |
|![19_psu](img/asm/pc_case/prt/19_psu.svg)           | M4-ATX Power Supply | 1   |

## II. Assembly Steps

1. Insert the ***DC Power Jack*** and the ***Motor*** and ***PC switches*** in their respective holes in the
***Front-Left case*** side.

    ![s001](img/asm/pc_case/stp/S001.svg)

2. Attach the case's ***Front Panel*** to the ***PCI corner*** piece with care, since the thin tine that divides the PCI
card slots can be very fragile. Then proceed to attach an ***Angle 1*** corner to the other end of the ***Front Panel***.

    ![s002](img/asm/pc_case/stp/S002.svg)

3. Place the ***PC's Port Cover*** in the ***Front Panel***'s opening from the inside.  

    ![s003](img/asm/pc_case/stp/S003.svg)

4. Using the *M3x8 Countersunk screws*, attach all of the ***Standoffs*** to the ***Bottom Plate*** as shown in the
picture.

     ![s004](img/asm/pc_case/stp/S004.svg)

5. Place the Front Panel, assembled in step **(3)**, along with the remaining ***Corner Pieces*** in their respective
positions on the ***Bottom Plate*** and secure them with *M4x12 Countersunk screws*.

     ![s005](img/asm/pc_case/stp/S005.svg) 
 
6. Position the assembled PC on top of the four ***10mm Standoffs***, by sliding it horizontaly so that the ports end in
their respective cutouts in the ***Port Cover***, and fasten it using four *M3x6 Cap screws*.
**(Note:** If a PCI card is attached to the motherboard, it may be blocking the access to one of the mounting holes. If that is the
case, detach the PCI card before screwing the motherboard to the base, and reattach it after the mounting screws have
been tightened.**)**

     ![s006](img/asm/pc_case/stp/S006.svg)

7. Affix the ***Car Interface Board*** to the ***10 mm Standoffs*** on the bottom plate using four *M3x6 Cap screws*.
It is a good idea to connect and arrange all the necessary cables to the PCB at this stage, since it will be covered in
the next step.

    ![s007](img/asm/pc_case/stp/S007.svg)
    
8.  Place the ***Power Supply Circuit*** on top of the longer ***40 mm Standoffs*** and secure it in place with four
*M3x6 Cap screws*. Finish all of the internal cabling and routing.

    ![s008](img/asm/pc_case/stp/S008.svg)

9. Slide the remaining ***Side Panels*** into their respective grooves.

    ![s009](img/asm/pc_case/stp/S009.svg)
   
10. Screw the ***Top Plate*** to the ***Corner Pieces*** using 6 *M4x12 Countersunk screws*.
    
    ![s010](img/asm/pc_case/stp/S010.svg)




## III. Exploded View

![exp](img/asm/pc_case/exp.svg)

#### BOM

| ID     | Description                | Qty |     | ID     | Description                | Qty |
|:------:|:---------------------------|:---:|:---:|:------:|:---------------------------|:---:|
|  **1** |Case Bottom                 | 1   |     | **13** |Standoff 10 mm              | 8   |
|  **2** |Case Corner 90º             | 1   |     | **14** |Standoff 40 mm              | 4   |
|  **3** |Case Corner PCI             | 1   |     | **15** |Toggle Switch w/LED         | 2   |
|  **4** |Case Corner Angle 1         | 2   |     | **16** |DC Power Jack               | 1   |
|  **5** |Case Corner Angle 2         | 2   |     | **17** |Mini-ITX PC                 | 1   |
|  **6** |Case Front                  | 1   |     | **18** |PC Port Plate               | 1   |
|  **7** |Case Back                   | 1   |     | **19** |M4-ATX Power Supply         | 1   |
|  **8** |Case Left                   | 1   |     | **20** |Car Interface Board         | 1   |
|  **9** |Case Right                  | 1   |     | **21** |M3x6 CBR Screws             | 12  |
| **10** |Case Front Left             | 1   |     | **22** |M3x8 CSK Screws             | 12  |
| **11** |Case Front Right            | 1   |     | **23** |M4x12 CSK Screws            | 12  |
| **12** |Case Top                    | 1   |     |        |                            |     |










