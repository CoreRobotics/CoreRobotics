function v = CR_RESULT_BAD_IC()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 8);
  end
  v = vInitialized;
end
