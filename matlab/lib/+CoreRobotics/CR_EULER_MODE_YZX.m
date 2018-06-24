function v = CR_EULER_MODE_YZX()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 14);
  end
  v = vInitialized;
end
