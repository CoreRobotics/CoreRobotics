function v = CR_INTEGRATION_RK4()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 39);
  end
  v = vInitialized;
end
