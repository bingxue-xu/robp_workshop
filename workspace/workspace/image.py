from PIL import Image, ImageTransform

image = Image.open('/home/rosuser/dd2419/workshop_ws/src/workspace/workspace/P_20250515_111603.jpg')
transform = [510, 1280, 5930, 670, 5880, 5580, 530, 4770]
image = image.transform((2000, 3000), ImageTransform.QuadTransform(
    transform), resample=Image.Resampling.BICUBIC)
image = image.rotate(180, expand=True)
image.show()

image.save('/home/rosuser/dd2419/workshop_ws/src/workspace/workspace/workspace_sim.gif')