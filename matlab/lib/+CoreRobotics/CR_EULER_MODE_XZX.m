function v = CR_EULER_MODE_XZX()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 11);
  end
  v = vInitialized;
end
