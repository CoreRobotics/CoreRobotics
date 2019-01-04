function v = CR_RESULT_SINGULAR()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 2);
  end
  v = vInitialized;
end
