function v = CR_EULER_MODE_YXZ()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 20);
  end
  v = vInitialized;
end
