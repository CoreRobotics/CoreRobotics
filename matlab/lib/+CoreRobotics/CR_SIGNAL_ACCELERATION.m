function v = CR_SIGNAL_ACCELERATION()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 3);
  end
  v = vInitialized;
end
