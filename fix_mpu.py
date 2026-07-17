with open('HardWare/MPU6050.c', 'r', encoding='utf-8') as f:
    c = f.read()

old = "    float gz_dps = ((float)gz_raw - gz_offset) * 250.0f / 32768.0f;\n        gz_filter = gz_filter * 0.8f + gz_dps * 0.2f;\n\n    if (fabsf(gz_filter) < 2.0f) gz_filter = 0.0f;\n\n    yaw_angle += gz_filter * dt;"

new = "    float gz_dps = ((float)gz_raw - gz_offset) * 250.0f / 32768.0f;\n\n    if (fabsf(gz_dps) < 1.5f) {\n        gz_filter = 0.0f;\n    } else {\n        gz_filter = gz_filter * 0.8f + gz_dps * 0.2f;\n    }\n\n    yaw_angle += gz_filter * dt;"

c = c.replace(old, new)

with open('HardWare/MPU6050.c', 'w', encoding='utf-8', newline='\n') as f:
    f.write(c)
print('Done')
