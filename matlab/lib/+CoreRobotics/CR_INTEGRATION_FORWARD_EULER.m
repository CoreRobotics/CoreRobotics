function v = CR_INTEGRATION_FORWARD_EULER()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 38);
  end
  v = vInitialized;
end
