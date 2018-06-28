function v = CR_RUN_STATE_STOPPED()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 6);
  end
  v = vInitialized;
end
