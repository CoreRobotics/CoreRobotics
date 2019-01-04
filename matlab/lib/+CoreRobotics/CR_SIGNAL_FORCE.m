function v = CR_SIGNAL_FORCE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 0);
  end
  v = vInitialized;
end
