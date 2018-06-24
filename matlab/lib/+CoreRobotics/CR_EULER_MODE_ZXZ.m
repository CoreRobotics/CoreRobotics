function v = CR_EULER_MODE_ZXZ()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 7);
  end
  v = vInitialized;
end
