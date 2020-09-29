try
V1_rotation.data(isnan(V1_rotation.data)) = 0;
V2_rotation.data(isnan(V2_rotation.data)) = 0;
V3_rotation.data(isnan(V3_rotation.data)) = 0;
V4_rotation.data(isnan(V4_rotation.data)) = 0;
V5_rotation.data(isnan(V5_rotation.data)) = 0;
V6_rotation.data(isnan(V6_rotation.data)) = 0;
V7_rotation.data(isnan(V7_rotation.data)) = 0;
V8_rotation.data(isnan(V8_rotation.data)) = 0;
V9_rotation.data(isnan(V9_rotation.data)) = 0;
V10_rotation.data(isnan(V10_rotation.data)) = 0;
catch
disp('Animation data not loaded')    
end