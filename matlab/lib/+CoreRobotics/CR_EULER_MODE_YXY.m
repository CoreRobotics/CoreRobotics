function v = CR_EULER_MODE_YXY()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 13);
  end
  v = vInitialized;
end
