function v = CR_RESULT_NOT_FOUND()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 4);
  end
  v = vInitialized;
end