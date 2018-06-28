function v = CR_MANIPULATOR_MODE_POSITION()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 34);
  end
  v = vInitialized;
end
