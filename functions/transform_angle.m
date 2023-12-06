function delta_angle = transform_angle(original_angle)

delta_angle = original_angle;
delta_angle(2) = -original_angle(2);
delta_angle(3) = -original_angle(3);
delta_angle(5) = -original_angle(5);
end