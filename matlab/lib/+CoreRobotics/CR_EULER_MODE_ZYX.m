function v = CR_EULER_MODE_ZYX()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 18);
  end
  v = vInitialized;
end
