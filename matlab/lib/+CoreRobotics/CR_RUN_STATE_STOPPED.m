function v = CR_RUN_STATE_STOPPED()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 5);
  end
  v = vInitialized;
end