function v = CR_RESULT_UNWRITABLE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 2);
  end
  v = vInitialized;
end
