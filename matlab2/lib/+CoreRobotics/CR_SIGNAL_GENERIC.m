function v = CR_SIGNAL_GENERIC()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 4);
  end
  v = vInitialized;
end
