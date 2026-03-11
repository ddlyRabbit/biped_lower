# URDF — Robot Descriptions

Two variants of the biped robot, both with **+X forward axis**.

## heavy/ (~30.7 kg)
- Original Onshape material densities
- 10 kg battery_chest (centered on torso)
- 12 DoF, 35 STL meshes
- Used for training with high-inertia stability

## light/ (~15.6 kg)
- Lighter material properties
- 0.5 kg battery_chest
- Same geometry, lower mass
- Primary variant for real hardware deployment

## Joint Configuration (12 DoF)

| Joint | Type | Per Leg |
|-------|------|---------|
| hip_pitch | RS04 (120Nm) | 1 |
| hip_roll | RS03 (60Nm) | 1 |
| hip_yaw | RS03 (60Nm) | 1 |
| knee | RS04 (120Nm) | 1 |
| foot_pitch | RS02 (17Nm) | 1 |
| foot_roll | RS02 (17Nm) | 1 |

## Notes
- Forward axis: **+X** (Onshape exports, March 2026)
- Ankle: parallel linkage (G1-style), 30Nm effort per joint in sim
- Battery sized to match torso footprint (edges aligned)
- Old -Y forward URDF available at tag `v1.0-neg-y-forward`
