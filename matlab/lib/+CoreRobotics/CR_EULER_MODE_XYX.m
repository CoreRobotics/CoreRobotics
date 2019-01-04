function v = CR_EULER_MODE_XYX()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 10);
  end
  v = vInitialized;
end
