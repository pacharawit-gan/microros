# meshes

Empty on purpose. The model currently uses primitive geometry (boxes/cylinders), which is
what you want while the dimensions are still being measured — primitives are fast, and a
wrong primitive is obviously wrong, whereas a wrong mesh looks convincing.

When you want a photo-real model, download Yahboom's **3D Model File** archive
(<https://www.yahboom.net/study/MicroROS-Pi5>, download table), export the parts to
`.dae` or `.stl`, drop them here, and swap the `<geometry>` blocks in
`../urdf/microros_pi5.urdf.xacro`:

```xml
<geometry>
  <mesh filename="package://microros_pi5_description/meshes/base_link.dae" scale="0.001 0.001 0.001"/>
</geometry>
```

Keep the `<collision>` blocks as primitives even then — mesh collisions are slow and buy
nothing for a differential-drive car.
