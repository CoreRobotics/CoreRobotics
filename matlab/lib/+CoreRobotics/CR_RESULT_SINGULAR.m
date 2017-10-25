function v = CR_RESULT_SINGULAR()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 6);
  end
  v = vInitialized;
end
