function v = CR_EULER_MODE_YZY()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 9);
  end
  v = vInitialized;
end
