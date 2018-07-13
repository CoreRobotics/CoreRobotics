function v = CR_RUN_STATE_PAUSED()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 8);
  end
  v = vInitialized;
end
