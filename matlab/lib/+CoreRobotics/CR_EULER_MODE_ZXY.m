function v = CR_EULER_MODE_ZXY()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 15);
  end
  v = vInitialized;
end
