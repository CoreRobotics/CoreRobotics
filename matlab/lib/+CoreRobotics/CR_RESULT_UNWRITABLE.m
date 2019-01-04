function v = CR_RESULT_UNWRITABLE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 3);
  end
  v = vInitialized;
end
