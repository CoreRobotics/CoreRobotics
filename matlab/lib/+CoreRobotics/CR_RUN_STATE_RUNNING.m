function v = CR_RUN_STATE_RUNNING()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 4);
  end
  v = vInitialized;
end
