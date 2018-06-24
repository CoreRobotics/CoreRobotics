function v = CR_RESULT_SUCCESS()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 0);
  end
  v = vInitialized;
end
