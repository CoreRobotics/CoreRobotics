function v = CR_MANAGER_SERVER()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 34);
  end
  v = vInitialized;
end
