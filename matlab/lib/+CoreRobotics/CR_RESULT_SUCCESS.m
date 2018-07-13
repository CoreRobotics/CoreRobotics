function v = CR_RESULT_SUCCESS()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 1);
  end
  v = vInitialized;
end
