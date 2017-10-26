function v = CR_SIGNAL_VELOCITY()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 2);
  end
  v = vInitialized;
end
