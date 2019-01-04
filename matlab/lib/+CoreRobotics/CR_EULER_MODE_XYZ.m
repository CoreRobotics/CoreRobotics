function v = CR_EULER_MODE_XYZ()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 15);
  end
  v = vInitialized;
end
