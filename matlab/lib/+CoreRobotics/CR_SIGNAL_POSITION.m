function v = CR_SIGNAL_POSITION()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 1);
  end
  v = vInitialized;
end
