function v = CR_EULER_MODE_YZX()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 16);
  end
  v = vInitialized;
end
