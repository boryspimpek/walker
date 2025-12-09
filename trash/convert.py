from urdf2mjcf import run

run(
    urdf_path = "Walker.urdf",
    mjcf_path = "Walkermu.xml",
    copy_meshes=True
)
