# 3D CAD Models & Mechanical Fabrication (.stl)

This directory contains the stereolithography (`.stl`) mesh models required to 3D print and assemble the physical direct-drive steering testbench.

---

## 1. Mechanical Component Breakdown

| Part ID | Component Name | Description | Recommended Material | Infill Density | File Name (.stl) |
| :---: | :--- | :--- | :--- | :---: | :--- |
| **CAD-01** | **Steering Wheel Rim** | 270 mm diameter ergonomic D-cut steering handwheel designed for high torsional stiffness under fast counter-steering | PETG / ABS / ASA | 50% Gyroid, 5 perimeters | `steering_wheel_rim_270mm.stl` |
| **CAD-02** | **Motor Hub Adapter** | Custom 4-bolt flange adapter coupling the BLDC hub motor axle/face to the 70 mm PCD steering wheel bolt circle | PETG / Polycarbonate (or CNC Al) | 100% Solid, 6 perimeters | `motor_hub_adapter_70pcd.stl` |
| **CAD-03** | **Bench Baseplate & Motor Mount** | Rigid L-bracket mounting plate securing the BLDC stator frame to 2020/2040 aluminum extrusion workbench profiles | PETG / Heavy-duty PLA+ | 60% Rectilinear, 5 walls | `motor_mount_bracket.stl` |
| **CAD-04** | **Optical Encoder Bracket** | Precision mounting arm positioning the 720 PPR incremental encoder parallel to the main rotor axis | PETG / PLA+ | 40% Gyroid, 4 walls | `encoder_mount_bracket.stl` |
| **CAD-05** | **Anti-Backlash Spur Gear (Motor)** | Mod 1.0, 40-teeth spur gear clamped to the rotating motor hub | PETG / Nylon (PA) | 100% Solid, 0.12 mm layer | `spur_gear_motor_40t_m1.stl` |
| **CAD-06** | **Anti-Backlash Spur Gear (Encoder)** | Mod 1.0, 40-teeth spur gear pressed onto the 6 mm optical encoder shaft (1:1 ratio) | PETG / Nylon (PA) | 100% Solid, 0.12 mm layer | `spur_gear_encoder_40t_m1.stl` |
| **CAD-07** | **Emergency Stop Enclosure** | External desktop housing for the 22 mm mushroom emergency cutoff pushbutton | PLA+ / PETG | 30% Grid | `estop_desk_enclosure.stl` |

---

## 2. Directory Structure

All ready-to-slice 3D models are stored in `hardware/cad/stl/`:

```text
hardware/cad/
├── README.md                 # Mechanical specifications and printing parameters
└── stl/                      # 3D stereolithography printable mesh files (.stl)
    ├── steering_wheel_rim_270mm.stl
    ├── motor_hub_adapter_70pcd.stl
    ├── motor_mount_bracket.stl
    ├── encoder_mount_bracket.stl
    ├── spur_gear_motor_40t_m1.stl
    ├── spur_gear_encoder_40t_m1.stl
    └── estop_desk_enclosure.stl
```

---

## 3. Recommended 3D Printing Specifications

To ensure mechanical integrity against dynamic torques up to $2.45\text{ N}\cdot\text{m}$ and high angular accelerations ($> 800\text{ rad/s}^2$):

- **Filament Type:** **PETG** or **ABS/ASA** (Standard PLA is acceptable for the emergency enclosure, but brittle under cyclic motor torque).
- **Layer Height:**
  - Structural brackets and wheel: `0.20 mm`
  - Spur gears: `0.12 mm` (ensures smooth involute tooth meshing and low friction)
- **Perimeters / Wall Line Count:** Minimum `5` walls (approx. $2.0\text{ mm}$ solid perimeter).
- **Top / Bottom Solid Layers:** Minimum `5` layers.
- **Infill Type:** Gyroid or Rectilinear (avoid Grid/Cubic which can delaminate under torsional shear).
- **Print Temperature:** $235\text{--}245^\circ\text{C}$ for PETG (bed at $75\text{--}85^\circ\text{C}$).

---

## 4. Hardware Fasteners & Assembly Guide

| Fastener | Quantity | Usage |
| :--- | :---: | :--- |
| **M4 × 20 mm Socket Head Cap Screw** | 4 | Attaches Steering Wheel Rim to Motor Hub Adapter |
| **M4 Nylon Insert Locknut** | 4 | Retains M4 screws on Hub Adapter |
| **M5 × 25 mm Hex Bolt & T-Nut** | 4 | Clamps Motor Mount Bracket to 2020/2040 Aluminum Extrusions |
| **M3 × 12 mm Socket Head Cap Screw** | 3 | Secures Optical Encoder to Encoder Mounting Bracket |
| **M3 × 8 mm Grub / Set Screw** | 2 | Locks 40T spur gears to D-shaft flats |

---

## 5. Mechanical Alignment Verification

1. **Gear Mesh Backlash:**
   - Mount the encoder bracket such that the 40T motor gear and 40T encoder gear mesh with zero perceptible backlash, but without radial binding.
   - Apply a small drop of PTFE dry lubricant or synthetic grease to the gear teeth.
2. **Torsional Runout:**
   - Rotate the wheel through $360^\circ$ by hand. Verify that the radial and axial runout at the rim is less than $1.0\text{ mm}$.
