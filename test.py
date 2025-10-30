import pyvista as pv
def on_pick(p): print("Picked:", p)
pl = pv.Plotter()
pl.add_mesh(pv.Sphere())
pl.enable_cell_picking(callback=on_pick)
pl.show()
